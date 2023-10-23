/*
   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

#include "sdkconfig.h"
#if CONFIG_LUA_RTOS_LUA_USE_ROBOTITO_SPP

#include "lua.h"
#include "lualib.h"
#include "lauxlib.h"
#include "error.h"
#include "sys.h"
#include "modules.h"
#include <sys/syslog.h>

#include <stdint.h>
#include <string.h>
#include <stdbool.h>
#include <stdio.h>
#include "nvs.h"
#include "nvs_flash.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/adds.h"
#include "freertos/ringbuf.h"
//#include "esp_log.h"
#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_gap_bt_api.h"
#include "esp_bt_device.h"
#include "esp_spp_api.h"

#include "time.h"
#include "sys/time.h"

#include "lua.h"
#include "lualib.h"
#include "lauxlib.h"

bool robotito_spp_initialized = false;
int robotito_spp_rcv_callback = LUA_REFNIL;
int robotito_spp_line_callback = LUA_REFNIL;

char *line_buff = NULL;
int line_buff_last = 0;

static uint8_t spp_data[ESP_SPP_MAX_MTU];
int spp_data_len = 0;

static bool is_connected = false;
uint32_t connection_handle;

#define SPP_TAG "SPP_ACCEPTOR_DEMO"
#define SPP_SERVER_NAME "SPP_SERVER"
#define EXCAMPLE_DEVICE_NAME "ESP_SPP_ACCEPTOR"
#define SPP_SHOW_DATA 0
#define SPP_SHOW_SPEED 1
#define SPP_SHOW_MODE SPP_SHOW_SPEED    /*Choose show mode: show data or speed*/

static const esp_spp_mode_t esp_spp_mode = ESP_SPP_MODE_CB;

static struct timeval time_new, time_old;
static long data_num = 0;

static const esp_spp_sec_t sec_mask = ESP_SPP_SEC_AUTHENTICATE;
static const esp_spp_role_t role_slave = ESP_SPP_ROLE_SLAVE;

//static xQueueHandle spp_rcv_queue = NULL;
#define STREAM_BUFFER_SIZE_BYTES 1028
RingbufHandle_t stream_buffer_handle;

static void print_speed(void)
{
    float time_old_s = time_old.tv_sec + time_old.tv_usec / 1000000.0;
    float time_new_s = time_new.tv_sec + time_new.tv_usec / 1000000.0;
    float time_interval = time_new_s - time_old_s;
    float speed = data_num * 8 / time_interval / 1000.0;
    syslog(LOG_INFO, "speed(%fs ~ %fs): %f kbit/s" , time_old_s, time_new_s, speed);
    data_num = 0;
    time_old.tv_sec = time_new.tv_sec;
    time_old.tv_usec = time_new.tv_usec;
}


void spp_rcv_task(void * arg)
{   
    for(;;){
        vTaskDelay(50 / portTICK_PERIOD_MS);
        
		size_t item_size;
	    char *item = (char *)xRingbufferReceiveUpTo(stream_buffer_handle, 
	    		&item_size, 
	    		portMAX_DELAY, 
	    		STREAM_BUFFER_SIZE_BYTES);
       
		if (item != NULL) {
			if (robotito_spp_rcv_callback!=LUA_REFNIL) {	
				//prepare thread
				lua_State *L = pvGetLuaState();
				lua_State *TL = lua_newthread(L);
				int tref = luaL_ref(L, LUA_REGISTRYINDEX);
				lua_rawgeti(L, LUA_REGISTRYINDEX, robotito_spp_rcv_callback);
				lua_xmove(L, TL, 1);

				lua_pushlstring(TL, item, item_size);
			    vRingbufferReturnItem(stream_buffer_handle, (void *)item);

				int status = lua_pcall(TL, 1, 0, 0);
				luaL_unref(TL, LUA_REGISTRYINDEX, tref);

				if (status != LUA_OK) {
					const char *msg = lua_tostring(TL, -1);
					lua_writestringerror("error in rcv callback: %s\n", msg);
					lua_pop(TL, 1);
				}
			}
	        if (robotito_spp_line_callback!=LUA_REFNIL) {
		        if (line_buff_last+item_size>CONFIG_ROBOTITO_SPP_LINEBUFFER) {
		            // if buffer overflow, send current buffer in error output
		            //prepare thread
					lua_State *L = pvGetLuaState();
					lua_State *TL = lua_newthread(L);
					int tref = luaL_ref(L, LUA_REGISTRYINDEX);
					lua_rawgeti(L, LUA_REGISTRYINDEX, robotito_spp_line_callback);
					lua_xmove(L, TL, 1);

		            lua_pushnil(TL);
					lua_pushlstring(TL, (char*)line_buff, line_buff_last);
					line_buff_last = 0;
		            int status = lua_pcall(TL, 2, 0, 0);
		            luaL_unref(TL, LUA_REGISTRYINDEX, tref);

		            if (status != LUA_OK) {
				        const char *msg = lua_tostring(TL, -1);
				        lua_writestringerror("error in line callback: %s\n", msg);
				        lua_pop(TL, 1);
					}    
					
		        }
		        memcpy(line_buff+line_buff_last, item, item_size);
		        int start_search = line_buff_last;
		        line_buff_last += item_size;
		        char *pos = memchr(line_buff+start_search, (char)10, line_buff_last-start_search);
		        while ( pos ) {
		            
		            //prepare thread
					lua_State *L = pvGetLuaState();
					lua_State *TL = lua_newthread(L);
					int tref = luaL_ref(L, LUA_REGISTRYINDEX);
					lua_rawgeti(L, LUA_REGISTRYINDEX, robotito_spp_line_callback);
					lua_xmove(L, TL, 1);

					lua_pushlstring(TL, line_buff, pos-line_buff);
					memcpy(line_buff, pos+1, line_buff+line_buff_last-pos-1);
		            int status = lua_pcall(TL, 1, 0, 0);
		            luaL_unref(TL, LUA_REGISTRYINDEX, tref);

		            if (status != LUA_OK) {
				        const char *msg = lua_tostring(TL, -1);
				        lua_writestringerror("error in line callback: %s\n", msg);
				        lua_pop(TL, 1);
					}                        
		            
		            line_buff_last -= (pos-line_buff+1);
		            pos = memchr(line_buff, (char)10, line_buff_last);
		        }
		    }

        } else {
        	//Failed to receive item
        	printf("Failed to receive item\n");
    	}
    }
    vTaskDelete(NULL);
}


static void esp_spp_cb(esp_spp_cb_event_t event, esp_spp_cb_param_t *param)
{
    switch (event) {
    case ESP_SPP_INIT_EVT:
        syslog(LOG_INFO, "ESP_SPP_INIT_EVT");
        esp_bt_dev_set_device_name(EXCAMPLE_DEVICE_NAME);
        esp_bt_gap_set_scan_mode(ESP_BT_SCAN_MODE_CONNECTABLE_DISCOVERABLE);
        esp_spp_start_srv(sec_mask,role_slave, 0, SPP_SERVER_NAME);
      
        break;
    case ESP_SPP_DISCOVERY_COMP_EVT:
        syslog(LOG_INFO, "ESP_SPP_DISCOVERY_COMP_EVT");
        break;
    case ESP_SPP_OPEN_EVT:
        syslog(LOG_INFO, "ESP_SPP_OPEN_EVT");
        break;
    case ESP_SPP_CLOSE_EVT:
        syslog(LOG_INFO, "ESP_SPP_CLOSE_EVT");
        is_connected = false;
        break;
    case ESP_SPP_START_EVT:
        syslog(LOG_INFO, "ESP_SPP_START_EVT");
        break;
    case ESP_SPP_CL_INIT_EVT:
        syslog(LOG_INFO, "ESP_SPP_CL_INIT_EVT");
        break;
    case ESP_SPP_DATA_IND_EVT:

        gettimeofday(&time_new, NULL);
        data_num += param->data_ind.len;
        if (time_new.tv_sec - time_old.tv_sec >= 3) {
            print_speed();
        }

	    UBaseType_t res =  xRingbufferSend(stream_buffer_handle, 
	    			( void * ) param->data_ind.data, 
	    			param->data_ind.len, 
	    			pdMS_TO_TICKS(100));
	    if (res != pdTRUE) {
	        printf("Failed to send item\n"); //TODO
	    }        
        
        break;
    case ESP_SPP_CONG_EVT:
        syslog(LOG_INFO, "ESP_SPP_CONG_EVT");
        if (param->cong.cong == 0) {
            esp_spp_write(param->cong.handle, spp_data_len, spp_data);
        }
        break;
    case ESP_SPP_WRITE_EVT:
        syslog(LOG_INFO, "ESP_SPP_WRITE_EVT");
        spp_data_len = 0;
        break;
    case ESP_SPP_SRV_OPEN_EVT:
        syslog(LOG_INFO, "ESP_SPP_SRV_OPEN_EVT");
        is_connected = true;
        connection_handle = param->open.handle;
        gettimeofday(&time_old, NULL);
        break;
    default:
        break;
    }
}

void esp_bt_gap_cb(esp_bt_gap_cb_event_t event, esp_bt_gap_cb_param_t *param)
{
    switch (event) {
    case ESP_BT_GAP_AUTH_CMPL_EVT:{
        if (param->auth_cmpl.stat == ESP_BT_STATUS_SUCCESS) {
            syslog(LOG_INFO, "authentication success: %s", param->auth_cmpl.device_name);
            //esp_log_buffer_hex(SPP_TAG, param->auth_cmpl.bda, ESP_BD_ADDR_LEN);
            printf("authentication success (%d bytes): ", ESP_BD_ADDR_LEN);
			for (int i=0; i<ESP_BD_ADDR_LEN; i++ ) {
				printf("%x", param->auth_cmpl.bda[i]);
			}
			printf("\n");
        } else {
            syslog(LOG_ERR,  "authentication failed, status:%d", param->auth_cmpl.stat);
        }
        break;
    }
    case ESP_BT_GAP_PIN_REQ_EVT:{
        syslog(LOG_INFO, "ESP_BT_GAP_PIN_REQ_EVT min_16_digit:%d", param->pin_req.min_16_digit);
        if (param->pin_req.min_16_digit) {
            syslog(LOG_INFO, "Input pin code: 0000 0000 0000 0000");
            esp_bt_pin_code_t pin_code = {0};
            esp_bt_gap_pin_reply(param->pin_req.bda, true, 16, pin_code);
        } else {
            syslog(LOG_INFO, "Input pin code: 1234");
            esp_bt_pin_code_t pin_code;
            pin_code[0] = '1';
            pin_code[1] = '2';
            pin_code[2] = '3';
            pin_code[3] = '4';
            esp_bt_gap_pin_reply(param->pin_req.bda, true, 4, pin_code);
        }
        break;
    }

#if (CONFIG_BT_SSP_ENABLED == true)
    case ESP_BT_GAP_CFM_REQ_EVT:
        syslog(LOG_INFO, "ESP_BT_GAP_CFM_REQ_EVT Please compare the numeric value: %d", param->cfm_req.num_val);
        esp_bt_gap_ssp_confirm_reply(param->cfm_req.bda, true);
        break;
    case ESP_BT_GAP_KEY_NOTIF_EVT:
        syslog(LOG_INFO, "ESP_BT_GAP_KEY_NOTIF_EVT passkey:%d", param->key_notif.passkey);
        break;
    case ESP_BT_GAP_KEY_REQ_EVT:
        syslog(LOG_INFO, "ESP_BT_GAP_KEY_REQ_EVT Please enter passkey!");
        break;
#endif

    default: {
        syslog(LOG_INFO, "event: %d", event);
        break;
    }
    }
    return;
}

static int robotito_spp_init (lua_State *L) {
	if (robotito_spp_initialized) {
	    syslog(LOG_WARNING, "already initialized\n");
        lua_pushnil(L);
        lua_pushstring(L, "already initialized");
        return 2;
	}
	
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK( ret );

    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_BLE));

    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    if ((ret = esp_bt_controller_init(&bt_cfg)) != ESP_OK) {
        syslog(LOG_ERR,  "%s initialize controller failed: %s\n", __func__, esp_err_to_name(ret));
        lua_pushnil(L);
        lua_pushstring(L, "initialize controller failed");
        return 2;
    }

    if ((ret = esp_bt_controller_enable(ESP_BT_MODE_CLASSIC_BT)) != ESP_OK) {
        syslog(LOG_ERR,  "%s enable controller failed: %s\n", __func__, esp_err_to_name(ret));
        lua_pushnil(L);
        lua_pushstring(L, "enable controller failed");
        return 2;
    }

    if ((ret = esp_bluedroid_init()) != ESP_OK) {
        syslog(LOG_ERR,  "%s initialize bluedroid failed: %s\n", __func__, esp_err_to_name(ret));
        lua_pushnil(L);
        lua_pushstring(L, "initialize bluedroid failed");
        return 2;
    }

    if ((ret = esp_bluedroid_enable()) != ESP_OK) {
        syslog(LOG_ERR,  "%s enable bluedroid failed: %s\n", __func__, esp_err_to_name(ret));
        lua_pushnil(L);
        lua_pushstring(L, "enable bluedroid failed");
        return 2;
    }

    if ((ret = esp_bt_gap_register_callback(esp_bt_gap_cb)) != ESP_OK) {
        syslog(LOG_ERR,  "%s gap register failed: %s\n", __func__, esp_err_to_name(ret));
        lua_pushnil(L);
        lua_pushstring(L, "gap register failed");
        return 2;
    }

    if ((ret = esp_spp_register_callback(esp_spp_cb)) != ESP_OK) {
        syslog(LOG_ERR,  "%s spp register failed: %s\n", __func__, esp_err_to_name(ret));
        lua_pushnil(L);
        lua_pushstring(L, "spp register failed");
        return 2;
    }

    if ((ret = esp_spp_init(esp_spp_mode)) != ESP_OK) {
        syslog(LOG_ERR,  "%s spp init failed: %s\n", __func__, esp_err_to_name(ret));
        lua_pushnil(L);
        lua_pushstring(L, "spp init failed");
        return 2;
    }

#if (CONFIG_BT_SSP_ENABLED == true)
    /* Set default parameters for Secure Simple Pairing */
    esp_bt_sp_param_t param_type = ESP_BT_SP_IOCAP_MODE;
    esp_bt_io_cap_t iocap = ESP_BT_IO_CAP_IO;
    esp_bt_gap_set_security_param(param_type, &iocap, sizeof(uint8_t));
#endif

    //spp_rcv_queue = xQueueCreate(1, sizeof(uint32_t));
    xTaskCreate(spp_rcv_task, "spp_rcv_task", 4096, NULL, 10, NULL);
    //Create ring buffer
    stream_buffer_handle = xRingbufferCreate(STREAM_BUFFER_SIZE_BYTES, RINGBUF_TYPE_BYTEBUF);
    if (stream_buffer_handle == NULL) {
        syslog(LOG_ERR,  "%s failed to create ring buffer\n", __func__);
        lua_pushnil(L);
        lua_pushstring(L, "failed to create ring buffer");
        return 2;
    }
    

    /*
     * Set default parameters for Legacy Pairing
     * Use variable pin, input pin code when pairing
     */
    esp_bt_pin_type_t pin_type = ESP_BT_PIN_TYPE_VARIABLE;
    esp_bt_pin_code_t pin_code;
    esp_bt_gap_set_pin(pin_type, 0, pin_code);
    
 	robotito_spp_initialized =  true;
 
    lua_pushboolean(L, true);
    return 1;
}

static int robotito_spp_send (lua_State *L) {
    if(!is_connected){
        syslog(LOG_ERR, "%s not connected\n", __func__);
        lua_pushnil(L);
        lua_pushstring(L, "not connected");
        return 2;
    }
    
    /*
    for (int i = 0; i < SPP_DATA_LEN; ++i) {
        spp_data[i] = i;
	}	
    esp_spp_write(param->srv_open.handle, spp_data_len, spp_data);
    */
	size_t length;
	const uint8_t *string = (uint8_t *) luaL_checklstring(L, 1, &length);
	
	if (length==0) {
	    lua_pushboolean(L, true);
    	return 1;
	}
	
	size_t available_len = ESP_SPP_MAX_MTU - spp_data_len;
	
	if (available_len < length) {
		memcpy(spp_data+spp_data_len,string, available_len);
		spp_data_len = spp_data_len+available_len; //ESP_SPP_MAX_MTU;
		esp_spp_write(connection_handle, spp_data_len, spp_data);

        lua_pushnil(L);
        lua_pushstring(L, "congestion");
	    lua_pushlstring(L, (char*)(string+available_len), length-available_len);
	    return 3;
	} else {
		memcpy(spp_data+spp_data_len,string, length);
		spp_data_len = spp_data_len+length; // <=ESP_SPP_MAX_MTU;
		esp_spp_write(connection_handle, spp_data_len, spp_data);
		
	    lua_pushboolean(L, true);
    	return 1;
	}
}

static int robotito_spp_rcv (lua_State *L) {
    bool enable = lua_toboolean(L, 1);
    if (enable) {
	    luaL_checktype(L, 1, LUA_TFUNCTION);
        lua_pushvalue(L, 1);
        robotito_spp_rcv_callback = luaL_ref(L, LUA_REGISTRYINDEX);
    } else {
        if (robotito_spp_rcv_callback==LUA_REFNIL) {
            lua_pushnil(L);
            lua_pushstring(L, "no rcv callback set");
            return 2;
        }
        robotito_spp_rcv_callback = LUA_REFNIL;
    }

    lua_pushboolean(L, true);
	return 1;
}

static int robotito_spp_line (lua_State *L) {
    bool enable = lua_toboolean(L, 1);
    if (enable) {
        if (line_buff==NULL) {
            line_buff = (char*)malloc(sizeof(char)*CONFIG_ROBOTITO_SPP_LINEBUFFER);
            if(line_buff == NULL){
                syslog(LOG_ERR, "%s malloc failed\n", __func__);
                lua_pushnil(L);
                lua_pushstring(L, "malloc failed");
                return 2;
            }   
        }

	    luaL_checktype(L, 1, LUA_TFUNCTION);
        lua_pushvalue(L, 1);
        robotito_spp_line_callback = luaL_ref(L, LUA_REGISTRYINDEX);
    } else {
        if (robotito_spp_line_callback==LUA_REFNIL) {
            lua_pushnil(L);
            lua_pushstring(L, "no line callback set");
            return 2;
        }
        robotito_spp_line_callback = LUA_REFNIL;
    }

    lua_pushboolean(L, true);
	return 1;
}


static const luaL_Reg robotito_spp[] = {
    {"init", robotito_spp_init},
    {"send", robotito_spp_send},
    {"set_rcv_callback", robotito_spp_rcv},
    {"set_line_callback", robotito_spp_line},
    {NULL, NULL}
};

LUALIB_API int luaopen_robotito_spp( lua_State *L ) {
    luaL_newlib(L, robotito_spp);
    return 1;
}

MODULE_REGISTER_RAM(ROBOTITO_SPP, robotito_spp, luaopen_robotito_spp, 1);


#endif
