/*
 * Copyright (C) 2015 - 2020, IBEROXARXA SERVICIOS INTEGRALES, S.L.
 * Copyright (C) 2015 - 2020, Jaume Olivé Petrus (jolive@whitecatboard.org)
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:º
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the <organization> nor the
 *       names of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 *     * The WHITECAT logotype cannot be changed, you can remove it, but you
 *       cannot change it in any way. The WHITECAT logotype is:
 *
 *          /\       /\
 *         /  \_____/  \
 *        /_____________\
 *        W H I T E C A T
 *
 *     * Redistributions in binary form must retain all copyright notices printed
 *       to any local or remote output device. This include any reference to
 *       Lua RTOS, whitecatboard.org, Lua, and other copyright notices that may
 *       appear in the future.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * Lua RTOS, Lua robotito BLE module *
 */

#include "sdkconfig.h"
#if CONFIG_LUA_RTOS_LUA_USE_ROBOTITO_BLE

#include "lua.h"
#include "lualib.h"
#include "lauxlib.h"
#include "error.h"
#include "sys.h"
#include "modules.h"
#include <sys/syslog.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "freertos/adds.h"
#include "esp_system.h"
//#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_bt.h"
#include "string.h"
#include "freertos/ringbuf.h"

#include "esp_gap_ble_api.h"
#include "esp_gatts_api.h"
#include "esp_bt_defs.h"
#include "esp_bt_main.h"
#include "robotito_ble.h"

#include "lua.h"
#include "lualib.h"
#include "lauxlib.h"

bool robotito_ble_initialized = false;
int robotito_ble_rcv_callback = LUA_REFNIL;
int robotito_ble_line_callback = LUA_REFNIL;

char *line_buff = NULL;
int line_buff_last = 0;

#define GATTS_TABLE_TAG  "GATTS_SPP_DEMO"

#define SPP_PROFILE_NUM             1
#define SPP_PROFILE_APP_IDX         0
#define ESP_SPP_APP_ID              0x56
#define SAMPLE_DEVICE_NAME          "ROBOTITO_SPP_SERVER"
#define SPP_SVC_INST_ID	            0

const char *ble_device_name = NULL;

/// SPP Service
static const uint16_t spp_service_uuid = 0xABF0;
/// Characteristic UUID
#define ESP_GATT_UUID_SPP_DATA_RECEIVE      0xABF1
#define ESP_GATT_UUID_SPP_DATA_NOTIFY       0xABF2
#define ESP_GATT_UUID_SPP_COMMAND_RECEIVE   0xABF3
#define ESP_GATT_UUID_SPP_COMMAND_NOTIFY    0xABF4

#ifdef SUPPORT_HEARTBEAT
#define ESP_GATT_UUID_SPP_HEARTBEAT         0xABF5
#endif

//max length 31
/*
static const uint8_t spp_adv_data[23] = {
    0x02,0x01,0x06,
    0x03,0x03,0xF0,0xAB,
    0x0F,0x09,0x45,0x53,0x50,0x5f,0x53,0x50,0x50,0x5f,0x53,0x45,0x52,0x56,0x45,0x52
};
*/
uint8_t *spp_adv_data = NULL;
size_t spp_adv_data_size = 0;


static uint16_t spp_mtu_size = 23;
static uint16_t spp_conn_id = 0xffff;
static esp_gatt_if_t spp_gatts_if = 0xff;
static xQueueHandle cmd_cmd_queue = NULL;

#define STREAM_BUFFER_SIZE_BYTES CONFIG_ROBOTITO_BLE_LINEBUFFER
RingbufHandle_t stream_buffer_handle;

#ifdef SUPPORT_HEARTBEAT
static xQueueHandle cmd_heartbeat_queue = NULL;
static uint8_t  heartbeat_s[9] = {'E','s','p','r','e','s','s','i','f'};
static bool enable_heart_ntf = false;
static uint8_t heartbeat_count_num = 0;
#endif

static bool enable_data_ntf = false;
static bool is_connected = false;
static esp_bd_addr_t spp_remote_bda = {0x0,};

static uint16_t spp_handle_table[SPP_IDX_NB];

static esp_ble_adv_params_t spp_adv_params = {
    .adv_int_min        = 0x20,
    .adv_int_max        = 0x40,
    .adv_type           = ADV_TYPE_IND,
    .own_addr_type      = BLE_ADDR_TYPE_PUBLIC,
    .channel_map        = ADV_CHNL_ALL,
    .adv_filter_policy  = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
};

struct gatts_profile_inst {
    esp_gatts_cb_t gatts_cb;
    uint16_t gatts_if;
    uint16_t app_id;
    uint16_t conn_id;
    uint16_t service_handle;
    esp_gatt_srvc_id_t service_id;
    uint16_t char_handle;
    esp_bt_uuid_t char_uuid;
    esp_gatt_perm_t perm;
    esp_gatt_char_prop_t property;
    uint16_t descr_handle;
    esp_bt_uuid_t descr_uuid;
};

typedef struct spp_receive_data_node{
    int32_t len;
    uint8_t * node_buff;
    struct spp_receive_data_node * next_node;
}spp_receive_data_node_t;

static spp_receive_data_node_t * temp_spp_recv_data_node_p1 = NULL;
static spp_receive_data_node_t * temp_spp_recv_data_node_p2 = NULL;

typedef struct spp_receive_data_buff{
    int32_t node_num;
    int32_t buff_size;
    spp_receive_data_node_t * first_node;
}spp_receive_data_buff_t;

static spp_receive_data_buff_t SppRecvDataBuff = {
    .node_num   = 0,
    .buff_size  = 0,
    .first_node = NULL
};

static void gatts_profile_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param);

/* One gatt-based profile one app_id and one gatts_if, this array will store the gatts_if returned by ESP_GATTS_REG_EVT */
static struct gatts_profile_inst spp_profile_tab[SPP_PROFILE_NUM] = {
    [SPP_PROFILE_APP_IDX] = {
        .gatts_cb = gatts_profile_event_handler,
        .gatts_if = ESP_GATT_IF_NONE,       /* Not get the gatt_if, so initial is ESP_GATT_IF_NONE */
    },
};

/*
 *  SPP PROFILE ATTRIBUTES
 ****************************************************************************************
 */

#define CHAR_DECLARATION_SIZE   (sizeof(uint8_t))
static const uint16_t primary_service_uuid = ESP_GATT_UUID_PRI_SERVICE;
static const uint16_t character_declaration_uuid = ESP_GATT_UUID_CHAR_DECLARE;
static const uint16_t character_client_config_uuid = ESP_GATT_UUID_CHAR_CLIENT_CONFIG;

static const uint8_t char_prop_read_notify = ESP_GATT_CHAR_PROP_BIT_READ|ESP_GATT_CHAR_PROP_BIT_NOTIFY;
static const uint8_t char_prop_read_write = ESP_GATT_CHAR_PROP_BIT_WRITE_NR|ESP_GATT_CHAR_PROP_BIT_READ;

#ifdef SUPPORT_HEARTBEAT
static const uint8_t char_prop_read_write_notify = ESP_GATT_CHAR_PROP_BIT_READ|ESP_GATT_CHAR_PROP_BIT_WRITE_NR|ESP_GATT_CHAR_PROP_BIT_NOTIFY;
#endif

///SPP Service - data receive characteristic, read&write without response
static const uint16_t spp_data_receive_uuid = ESP_GATT_UUID_SPP_DATA_RECEIVE;
static const uint8_t  spp_data_receive_val[20] = {0x00};

///SPP Service - data notify characteristic, notify&read
static const uint16_t spp_data_notify_uuid = ESP_GATT_UUID_SPP_DATA_NOTIFY;
static const uint8_t  spp_data_notify_val[20] = {0x00};
static const uint8_t  spp_data_notify_ccc[2] = {0x00, 0x00};

///SPP Service - command characteristic, read&write without response
static const uint16_t spp_command_uuid = ESP_GATT_UUID_SPP_COMMAND_RECEIVE;
static const uint8_t  spp_command_val[10] = {0x00};

///SPP Service - status characteristic, notify&read
static const uint16_t spp_status_uuid = ESP_GATT_UUID_SPP_COMMAND_NOTIFY;
static const uint8_t  spp_status_val[10] = {0x00};
static const uint8_t  spp_status_ccc[2] = {0x00, 0x00};

#ifdef SUPPORT_HEARTBEAT
///SPP Server - Heart beat characteristic, notify&write&read
static const uint16_t spp_heart_beat_uuid = ESP_GATT_UUID_SPP_HEARTBEAT;
static const uint8_t  spp_heart_beat_val[2] = {0x00, 0x00};
static const uint8_t  spp_heart_beat_ccc[2] = {0x00, 0x00};
#endif

///Full HRS Database Description - Used to add attributes into the database
static const esp_gatts_attr_db_t spp_gatt_db[SPP_IDX_NB] =
{
    //SPP -  Service Declaration
    [SPP_IDX_SVC]                      	=
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&primary_service_uuid, ESP_GATT_PERM_READ,
    sizeof(spp_service_uuid), sizeof(spp_service_uuid), (uint8_t *)&spp_service_uuid}},

    //SPP -  data receive characteristic Declaration
    [SPP_IDX_SPP_DATA_RECV_CHAR]            =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ,
    CHAR_DECLARATION_SIZE,CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_read_write}},

    //SPP -  data receive characteristic Value
    [SPP_IDX_SPP_DATA_RECV_VAL]             	=
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&spp_data_receive_uuid, ESP_GATT_PERM_READ|ESP_GATT_PERM_WRITE,
    SPP_DATA_MAX_LEN,sizeof(spp_data_receive_val), (uint8_t *)spp_data_receive_val}},

    //SPP -  data notify characteristic Declaration
    [SPP_IDX_SPP_DATA_NOTIFY_CHAR]  =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ,
    CHAR_DECLARATION_SIZE,CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_read_notify}},

    //SPP -  data notify characteristic Value
    [SPP_IDX_SPP_DATA_NTY_VAL]   =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&spp_data_notify_uuid, ESP_GATT_PERM_READ,
    SPP_DATA_MAX_LEN, sizeof(spp_data_notify_val), (uint8_t *)spp_data_notify_val}},

    //SPP -  data notify characteristic - Client Characteristic Configuration Descriptor
    [SPP_IDX_SPP_DATA_NTF_CFG]         =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_client_config_uuid, ESP_GATT_PERM_READ|ESP_GATT_PERM_WRITE,
    sizeof(uint16_t),sizeof(spp_data_notify_ccc), (uint8_t *)spp_data_notify_ccc}},

    //SPP -  command characteristic Declaration
    [SPP_IDX_SPP_COMMAND_CHAR]            =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ,
    CHAR_DECLARATION_SIZE,CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_read_write}},

    //SPP -  command characteristic Value
    [SPP_IDX_SPP_COMMAND_VAL]                 =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&spp_command_uuid, ESP_GATT_PERM_READ|ESP_GATT_PERM_WRITE,
    SPP_CMD_MAX_LEN,sizeof(spp_command_val), (uint8_t *)spp_command_val}},

    //SPP -  status characteristic Declaration
    [SPP_IDX_SPP_STATUS_CHAR]            =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ,
    CHAR_DECLARATION_SIZE,CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_read_notify}},

    //SPP -  status characteristic Value
    [SPP_IDX_SPP_STATUS_VAL]                 =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&spp_status_uuid, ESP_GATT_PERM_READ,
    SPP_STATUS_MAX_LEN,sizeof(spp_status_val), (uint8_t *)spp_status_val}},

    //SPP -  status characteristic - Client Characteristic Configuration Descriptor
    [SPP_IDX_SPP_STATUS_CFG]         =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_client_config_uuid, ESP_GATT_PERM_READ|ESP_GATT_PERM_WRITE,
    sizeof(uint16_t),sizeof(spp_status_ccc), (uint8_t *)spp_status_ccc}},

#ifdef SUPPORT_HEARTBEAT
    //SPP -  Heart beat characteristic Declaration
    [SPP_IDX_SPP_HEARTBEAT_CHAR]  =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ,
    CHAR_DECLARATION_SIZE,CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_read_write_notify}},

    //SPP -  Heart beat characteristic Value
    [SPP_IDX_SPP_HEARTBEAT_VAL]   =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&spp_heart_beat_uuid, ESP_GATT_PERM_READ|ESP_GATT_PERM_WRITE,
    sizeof(spp_heart_beat_val), sizeof(spp_heart_beat_val), (uint8_t *)spp_heart_beat_val}},

    //SPP -  Heart beat characteristic - Client Characteristic Configuration Descriptor
    [SPP_IDX_SPP_HEARTBEAT_CFG]         =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_client_config_uuid, ESP_GATT_PERM_READ|ESP_GATT_PERM_WRITE,
    sizeof(uint16_t),sizeof(spp_data_notify_ccc), (uint8_t *)spp_heart_beat_ccc}},
#endif
};

static uint8_t find_char_and_desr_index(uint16_t handle)
{
    uint8_t error = 0xff;

    for(int i = 0; i < SPP_IDX_NB ; i++){
        if(handle == spp_handle_table[i]){
            return i;
        }
    }

    return error;
}

static bool store_wr_buffer(esp_ble_gatts_cb_param_t *p_data)
{
    temp_spp_recv_data_node_p1 = (spp_receive_data_node_t *)malloc(sizeof(spp_receive_data_node_t));

    if(temp_spp_recv_data_node_p1 == NULL){
        syslog(LOG_INFO, "malloc error %s %d\n", __func__, __LINE__);
        return false;
    }
    if(temp_spp_recv_data_node_p2 != NULL){
        temp_spp_recv_data_node_p2->next_node = temp_spp_recv_data_node_p1;
    }
    temp_spp_recv_data_node_p1->len = p_data->write.len;
    SppRecvDataBuff.buff_size += p_data->write.len;
    temp_spp_recv_data_node_p1->next_node = NULL;
    temp_spp_recv_data_node_p1->node_buff = (uint8_t *)malloc(p_data->write.len);
    temp_spp_recv_data_node_p2 = temp_spp_recv_data_node_p1;
    memcpy(temp_spp_recv_data_node_p1->node_buff,p_data->write.value,p_data->write.len);
    if(SppRecvDataBuff.node_num == 0){
        SppRecvDataBuff.first_node = temp_spp_recv_data_node_p1;
        SppRecvDataBuff.node_num++;
    }else{
        SppRecvDataBuff.node_num++;
    }

    return true;
}

static void free_write_buffer(void)
{
    temp_spp_recv_data_node_p1 = SppRecvDataBuff.first_node;

    while(temp_spp_recv_data_node_p1 != NULL){
        temp_spp_recv_data_node_p2 = temp_spp_recv_data_node_p1->next_node;
        free(temp_spp_recv_data_node_p1->node_buff);
        free(temp_spp_recv_data_node_p1);
        temp_spp_recv_data_node_p1 = temp_spp_recv_data_node_p2;
    }

    SppRecvDataBuff.node_num = 0;
    SppRecvDataBuff.buff_size = 0;
    SppRecvDataBuff.first_node = NULL;
}

/*
static void print_write_buffer(void)
{
    temp_spp_recv_data_node_p1 = SppRecvDataBuff.first_node;

    while(temp_spp_recv_data_node_p1 != NULL){
        uart_write_bytes(UART_NUM_1, (char *)(temp_spp_recv_data_node_p1->node_buff), temp_spp_recv_data_node_p1->len);
        temp_spp_recv_data_node_p1 = temp_spp_recv_data_node_p1->next_node;
    }
}
*/


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
			if (robotito_ble_rcv_callback!=LUA_REFNIL) {	
				//prepare thread
				lua_State *L = pvGetLuaState();
				lua_State *TL = lua_newthread(L);
				int tref = luaL_ref(L, LUA_REGISTRYINDEX);
				lua_rawgeti(L, LUA_REGISTRYINDEX, robotito_ble_rcv_callback);
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
	        if (robotito_ble_line_callback!=LUA_REFNIL) {
		        if (line_buff_last+item_size>CONFIG_ROBOTITO_BLE_LINEBUFFER) {
		            // if buffer overflow, send current buffer in error output
		            //prepare thread
					lua_State *L = pvGetLuaState();
					lua_State *TL = lua_newthread(L);
					int tref = luaL_ref(L, LUA_REGISTRYINDEX);
					lua_rawgeti(L, LUA_REGISTRYINDEX, robotito_ble_line_callback);
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
					lua_rawgeti(L, LUA_REGISTRYINDEX, robotito_ble_line_callback);
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


#ifdef SUPPORT_HEARTBEAT
void spp_heartbeat_task(void * arg)
{
    uint16_t cmd_id;

    for(;;) {
        vTaskDelay(50 / portTICK_PERIOD_MS);
        if(xQueueReceive(cmd_heartbeat_queue, &cmd_id, portMAX_DELAY)) {
            while(1){
                heartbeat_count_num++;
                vTaskDelay(5000/ portTICK_PERIOD_MS);
                if((heartbeat_count_num >3)&&(is_connected)){
                    esp_ble_gap_disconnect(spp_remote_bda);
                }
                if(is_connected && enable_heart_ntf){
                    esp_ble_gatts_send_indicate(spp_gatts_if, spp_conn_id, spp_handle_table[SPP_IDX_SPP_HEARTBEAT_VAL],sizeof(heartbeat_s), heartbeat_s, false);
                }else if(!is_connected){
                    break;
                }
            }
        }
    }
    vTaskDelete(NULL);
}
#endif

void spp_cmd_task(void * arg)
{   
    uint8_t * cmd_id;

    for(;;){
        vTaskDelay(50 / portTICK_PERIOD_MS);
        if(xQueueReceive(cmd_cmd_queue, &cmd_id, portMAX_DELAY)) {
            //esp_log_buffer_char(GATTS_TABLE_TAG,(char *)(cmd_id),strlen((char *)cmd_id));
            printf("command: ");
            for (int i=0; i<strlen((char*)cmd_id); i++ ) {
	        	printf("%c", cmd_id[i]);
	        }
            printf("\n");

            free(cmd_id);
        }
    }
    vTaskDelete(NULL);
}

static void spp_task_init(void)
{
#ifdef SUPPORT_HEARTBEAT
    cmd_heartbeat_queue = xQueueCreate(10, sizeof(uint32_t));
    xTaskCreate(spp_heartbeat_task, "spp_heartbeat_task", 2048, NULL, 10, NULL);
#endif

    cmd_cmd_queue = xQueueCreate(10, sizeof(uint32_t));
    xTaskCreate(spp_cmd_task, "spp_cmd_task", 2048, NULL, 10, NULL);
}

static void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param)
{
    esp_err_t err;
    syslog(LOG_ERR, "GAP_EVT, event %d\n", event);

    switch (event) {
    case ESP_GAP_BLE_ADV_DATA_RAW_SET_COMPLETE_EVT:
        esp_ble_gap_start_advertising(&spp_adv_params);
        break;
    case ESP_GAP_BLE_ADV_START_COMPLETE_EVT:
        //advertising start complete event to indicate advertising start successfully or failed
        if((err = param->adv_start_cmpl.status) != ESP_BT_STATUS_SUCCESS) {
            syslog(LOG_ERR, "Advertising start failed: %s\n", esp_err_to_name(err));
        } else {
            syslog(LOG_ERR, "Advertising start success\n");
        }
        break;
    default:
        break;
    }
}

static void gatts_profile_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param)
{
    esp_ble_gatts_cb_param_t *p_data = (esp_ble_gatts_cb_param_t *) param;
    uint8_t res = 0xff;

    syslog(LOG_INFO, "event = %x\n",event);
    switch (event) {
    	case ESP_GATTS_REG_EVT:
    	    syslog(LOG_INFO, "%s %d\n", __func__, __LINE__);
    	    printf("calling esp_ble_gap_set_device_name with %s\n", ble_device_name);
        	esp_ble_gap_set_device_name(ble_device_name);

        	syslog(LOG_INFO, "%s %d\n", __func__, __LINE__);
        	esp_ble_gap_config_adv_data_raw((uint8_t *)spp_adv_data, spp_adv_data_size);

        	syslog(LOG_INFO, "%s %d\n", __func__, __LINE__);
        	esp_ble_gatts_create_attr_tab(spp_gatt_db, gatts_if, SPP_IDX_NB, SPP_SVC_INST_ID);
       	break;
    	case ESP_GATTS_READ_EVT:
            res = find_char_and_desr_index(p_data->read.handle);
            if(res == SPP_IDX_SPP_STATUS_VAL){
                //TODO:client read the status characteristic
            }
       	 break;
    	case ESP_GATTS_WRITE_EVT: {
    	    res = find_char_and_desr_index(p_data->write.handle);
            if(p_data->write.is_prep == false){
                syslog(LOG_INFO, "ESP_GATTS_WRITE_EVT : handle = %d\n", res);
                if(res == SPP_IDX_SPP_COMMAND_VAL){
                    uint8_t * spp_cmd_buff = NULL;
                    spp_cmd_buff = (uint8_t *)malloc((spp_mtu_size - 3) * sizeof(uint8_t));
                    if(spp_cmd_buff == NULL){
                        syslog(LOG_ERR, "%s malloc failed\n", __func__);
                        break;
                    }
                    memset(spp_cmd_buff,0x0,(spp_mtu_size - 3));
                    memcpy(spp_cmd_buff,p_data->write.value,p_data->write.len);
                    xQueueSend(cmd_cmd_queue,&spp_cmd_buff,10/portTICK_PERIOD_MS);
                }else if(res == SPP_IDX_SPP_DATA_NTF_CFG){
                    if((p_data->write.len == 2)&&(p_data->write.value[0] == 0x01)&&(p_data->write.value[1] == 0x00)){
                        enable_data_ntf = true;
                    }else if((p_data->write.len == 2)&&(p_data->write.value[0] == 0x00)&&(p_data->write.value[1] == 0x00)){
                        enable_data_ntf = false;
                    }
                }
#ifdef SUPPORT_HEARTBEAT
                else if(res == SPP_IDX_SPP_HEARTBEAT_CFG){
                    if((p_data->write.len == 2)&&(p_data->write.value[0] == 0x01)&&(p_data->write.value[1] == 0x00)){
                        enable_heart_ntf = true;
                    }else if((p_data->write.len == 2)&&(p_data->write.value[0] == 0x00)&&(p_data->write.value[1] == 0x00)){
                        enable_heart_ntf = false;
                    }
                }else if(res == SPP_IDX_SPP_HEARTBEAT_VAL){
                    if((p_data->write.len == sizeof(heartbeat_s))&&(memcmp(heartbeat_s,p_data->write.value,sizeof(heartbeat_s)) == 0)){
                        heartbeat_count_num = 0;
                    }
                }
#endif
                else if(res == SPP_IDX_SPP_DATA_RECV_VAL){
#ifdef SPP_DEBUG_MODE
                    //esp_log_buffer_char(GATTS_TABLE_TAG,(char *)(p_data->write.value),p_data->write.len);
                    printf("Arrived %d bytes: ", p_data->write.len);
                    for (int i=0; i<p_data->write.len; i++ ) {
						printf("%c", p_data->write.value[i]);
        			}
					printf("\n");
#endif
					UBaseType_t res =  xRingbufferSend(stream_buffer_handle, 
								( void * )p_data->write.value, 
								p_data->write.len, 
								pdMS_TO_TICKS(100));
					if (res != pdTRUE) {
						printf("Failed to send item\n"); //TODO
					}   


////////////////////////////////////
/*
                    if (robotito_ble_rcv_callback!=LUA_REFNIL) {

		                //prepare thread
						lua_State *L = pvGetLuaState();
						lua_State *TL = lua_newthread(L);
						int tref = luaL_ref(L, LUA_REGISTRYINDEX);
						lua_rawgeti(L, LUA_REGISTRYINDEX, robotito_ble_rcv_callback);
						lua_xmove(L, TL, 1);

						lua_pushlstring(TL, (char*)p_data->write.value, p_data->write.len);
                        int status = lua_pcall(TL, 1, 0, 0);
			            luaL_unref(TL, LUA_REGISTRYINDEX, tref);

		                if (status != LUA_OK) {
					        const char *msg = lua_tostring(TL, -1);
					        lua_writestringerror("error in rcv callback: %s\n", msg);
					        lua_pop(TL, 1);
						}
                    }
                    if (robotito_ble_line_callback!=LUA_REFNIL) {
                        if (line_buff_last+p_data->write.len>CONFIG_ROBOTITO_BLE_LINEBUFFER) {
                            // if buffer overflow, send current buffer in error output
		                    //prepare thread
						    lua_State *L = pvGetLuaState();
						    lua_State *TL = lua_newthread(L);
						    int tref = luaL_ref(L, LUA_REGISTRYINDEX);
						    lua_rawgeti(L, LUA_REGISTRYINDEX, robotito_ble_line_callback);
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
                        memcpy(line_buff+line_buff_last, p_data->write.value,p_data->write.len);
                        int start_search = line_buff_last;
                        line_buff_last += p_data->write.len;
                        char *pos = memchr(line_buff+start_search, (char)10, line_buff_last-start_search);
                        while ( pos ) {
                            
		                    //prepare thread
						    lua_State *L = pvGetLuaState();
						    lua_State *TL = lua_newthread(L);
						    int tref = luaL_ref(L, LUA_REGISTRYINDEX);
						    lua_rawgeti(L, LUA_REGISTRYINDEX, robotito_ble_line_callback);
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
                    */
                    /////////////////////////////////
                    
                }else{
                    //TODO:
                }
            }else if((p_data->write.is_prep == true)&&(res == SPP_IDX_SPP_DATA_RECV_VAL)){
                syslog(LOG_INFO, "ESP_GATTS_PREP_WRITE_EVT : handle = %d\n", res);
                store_wr_buffer(p_data);
            }
      	 	break;
    	}
    	case ESP_GATTS_EXEC_WRITE_EVT:{
    	    syslog(LOG_INFO, "ESP_GATTS_EXEC_WRITE_EVT\n");
    	    if(p_data->exec_write.exec_write_flag){
    	        //print_write_buffer();
    	        free_write_buffer();
    	    }
    	    break;
    	}
    	case ESP_GATTS_MTU_EVT:
    	    spp_mtu_size = p_data->mtu.mtu;
    	    break;
    	case ESP_GATTS_CONF_EVT:
    	    break;
    	case ESP_GATTS_UNREG_EVT:
        	break;
    	case ESP_GATTS_DELETE_EVT:
        	break;
    	case ESP_GATTS_START_EVT:
        	break;
    	case ESP_GATTS_STOP_EVT:
        	break;
    	case ESP_GATTS_CONNECT_EVT:
    	    spp_conn_id = p_data->connect.conn_id;
    	    spp_gatts_if = gatts_if;
    	    is_connected = true;
    	    memcpy(&spp_remote_bda,&p_data->connect.remote_bda,sizeof(esp_bd_addr_t));
#ifdef SUPPORT_HEARTBEAT
    	    uint16_t cmd = 0;
            xQueueSend(cmd_heartbeat_queue,&cmd,10/portTICK_PERIOD_MS);
#endif
        	break;
    	case ESP_GATTS_DISCONNECT_EVT:
    	    is_connected = false;
    	    enable_data_ntf = false;
#ifdef SUPPORT_HEARTBEAT
    	    enable_heart_ntf = false;
    	    heartbeat_count_num = 0;
#endif
    	    esp_ble_gap_start_advertising(&spp_adv_params);
    	    break;
    	case ESP_GATTS_OPEN_EVT:
    	    break;
    	case ESP_GATTS_CANCEL_OPEN_EVT:
    	    break;
    	case ESP_GATTS_CLOSE_EVT:
    	    break;
    	case ESP_GATTS_LISTEN_EVT:
    	    break;
    	case ESP_GATTS_CONGEST_EVT:
    	    break;
    	case ESP_GATTS_CREAT_ATTR_TAB_EVT:{
    	    syslog(LOG_INFO, "The number handle =%x\n",param->add_attr_tab.num_handle);
    	    if (param->add_attr_tab.status != ESP_GATT_OK){
    	        syslog(LOG_ERR, "Create attribute table failed, error code=0x%x", param->add_attr_tab.status);
    	    }
    	    else if (param->add_attr_tab.num_handle != SPP_IDX_NB){
    	        syslog(LOG_ERR, "Create attribute table abnormally, num_handle (%d) doesn't equal to HRS_IDX_NB(%d)", param->add_attr_tab.num_handle, SPP_IDX_NB);
    	    }
    	    else {
    	        memcpy(spp_handle_table, param->add_attr_tab.handles, sizeof(spp_handle_table));
    	        esp_ble_gatts_start_service(spp_handle_table[SPP_IDX_SVC]);
    	    }
    	    break;
    	}
    	default:
    	    break;
    }
}


static void gatts_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param)
{
    syslog(LOG_INFO, "EVT %d, gatts if %d\n", event, gatts_if);

    /* If event is register event, store the gatts_if for each profile */
    if (event == ESP_GATTS_REG_EVT) {
        if (param->reg.status == ESP_GATT_OK) {
            spp_profile_tab[SPP_PROFILE_APP_IDX].gatts_if = gatts_if;
        } else {
            syslog(LOG_INFO, "Reg app failed, app_id %04x, status %d\n",param->reg.app_id, param->reg.status);
            return;
        }
    }

    do {
        int idx;
        for (idx = 0; idx < SPP_PROFILE_NUM; idx++) {
            if (gatts_if == ESP_GATT_IF_NONE || /* ESP_GATT_IF_NONE, not specify a certain gatt_if, need to call every profile cb function */
                    gatts_if == spp_profile_tab[idx].gatts_if) {
                if (spp_profile_tab[idx].gatts_cb) {
                    spp_profile_tab[idx].gatts_cb(event, gatts_if, param);
                }
            }
        }
    } while (0);
}


static int robotito_ble_init (lua_State *L) {
	if (robotito_ble_initialized) {
	    syslog(LOG_WARNING, "already initialized\n");
        lua_pushnil(L);
        lua_pushstring(L, "already initialized");
        return 2;
	}

	printf("initializing robotito_ble\n");
    
 	const char* name = luaL_checkstring(L, 1);
	if (name) {
		ble_device_name = strdup(name);
	  	printf("  ble device name: %s \n", ble_device_name);
    } else {
		ble_device_name = SAMPLE_DEVICE_NAME;
	  	printf("  ble device name (default): %s \n", ble_device_name);
    }
    
  
  	/*  
    static const uint8_t spp_adv_data[23] = {
    0x02,0x01,0x06,
    0x03,0x03,0xF0,0xAB,
    0x0F,0x09,0x45,0x53,0x50,0x5f,0x53,0x50,0x50,0x5f,0x53,0x45,0x52,0x56,0x45,0x52
    };*/
    size_t ble_device_name_length = strlen(ble_device_name);
	spp_adv_data_size = 9+ble_device_name_length;
	if (spp_adv_data_size>31) {
	    syslog(LOG_ERR, "ble device name too long\n");
        lua_pushnil(L);
        lua_pushstring(L, "ble device name too long");
        return 2;
	} 
	spp_adv_data = malloc(sizeof(uint8_t)*spp_adv_data_size);
	spp_adv_data[0]=0x02; spp_adv_data[1]=0x01; spp_adv_data[2]=0x06;
	spp_adv_data[3]=0x03; spp_adv_data[4]=0x03; spp_adv_data[5]=0xF0; spp_adv_data[6]=0xAB;
	spp_adv_data[7]=ble_device_name_length+1; spp_adv_data[8]=0x09;
	memcpy(spp_adv_data+9, ble_device_name, ble_device_name_length);
	
	printf ("adv (len=%u):", spp_adv_data_size);
	for (size_t i=0; i<spp_adv_data_size; i++) {
	  printf(" %02X", spp_adv_data[i]);
	}
	printf ("\n");
  
	esp_err_t ret;
	esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();

	// Initialize NVS
	ret = nvs_flash_init();
	if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    	ESP_ERROR_CHECK(nvs_flash_erase());
	    ret = nvs_flash_init();
	}
	ESP_ERROR_CHECK( ret );

	ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));
	ret = esp_bt_controller_init(&bt_cfg);
	if (ret) {
	    syslog(LOG_ERR, "%s init controller failed: %s\n", __func__, esp_err_to_name(ret));
        lua_pushnil(L);
        lua_pushstring(L, "init controller failed");
        return 2;
	}

	ret = esp_bt_controller_enable(ESP_BT_MODE_BLE);
	if (ret) {
    	syslog(LOG_ERR, "%s enable controller failed: %s\n", __func__, esp_err_to_name(ret));
        lua_pushnil(L);
        lua_pushstring(L, "enable controller failed");
        return 2;
	}

	syslog(LOG_INFO, "%s init bluetooth\n", __func__);
	ret = esp_bluedroid_init();
	if (ret) {
	    syslog(LOG_ERR, "%s init bluetooth failed: %s\n", __func__, esp_err_to_name(ret));
        lua_pushnil(L);
        lua_pushstring(L, "init bluetooth failed");
        return 2;
	}
	ret = esp_bluedroid_enable();
	if (ret) {
    	syslog(LOG_ERR, "%s enable bluetooth failed: %s\n", __func__, esp_err_to_name(ret));
        lua_pushnil(L);
        lua_pushstring(L, "enable bluetooth failed");
        return 2;
	}
	
	esp_ble_gatts_register_callback(gatts_event_handler);
	esp_ble_gap_register_callback(gap_event_handler);
	esp_ble_gatts_app_register(ESP_SPP_APP_ID);

	//spp_rcv_queue = xQueueCreate(1, sizeof(uint32_t));
    xTaskCreate(spp_rcv_task, "spp_rcv_task", CONFIG_ROBOTITO_BLE_STACK_SIZE, NULL, 10, NULL);
    //Create ring buffer
    stream_buffer_handle = xRingbufferCreate(STREAM_BUFFER_SIZE_BYTES, RINGBUF_TYPE_BYTEBUF);
    if (stream_buffer_handle == NULL) {
        syslog(LOG_ERR,  "%s failed to create ring buffer\n", __func__);
        lua_pushnil(L);
        lua_pushstring(L, "failed to create ring buffer");
        return 2;
    }

	spp_task_init();
 
 	robotito_ble_initialized =  true;
 
    lua_pushboolean(L, true);
    return 1;
}

static int robotito_ble_send (lua_State *L) {
    if(!enable_data_ntf){
        syslog(LOG_ERR, "%s not enabled data Notify\n", __func__);
        lua_pushnil(L);
        lua_pushstring(L, "not enabled data Notify");
        return 2;
    }

    uint8_t total_num = 0;
    uint8_t current_num = 0;
    
	size_t length;
	const uint8_t *string = (uint8_t *) luaL_checklstring(L, 1, &length);

    uint8_t * temp = NULL;
    uint8_t * ntf_value_p = NULL;
    
    temp = (uint8_t *)malloc(sizeof(uint8_t)*length);
    
    if(temp == NULL){
        syslog(LOG_ERR, "%s malloc.1 failed\n", __func__);
        lua_pushnil(L);
        lua_pushstring(L, "malloc.1 failed");
    }
    memcpy(temp,string,length);

    if(length <= (spp_mtu_size - 3)){
        esp_ble_gatts_send_indicate(spp_gatts_if, spp_conn_id, spp_handle_table[SPP_IDX_SPP_DATA_NTY_VAL],length, temp, false);
    }else if(length > (spp_mtu_size - 3)){
        if((length%(spp_mtu_size - 3)) == 0){
            total_num = length/(spp_mtu_size - 3);
        }else{
            total_num = length/(spp_mtu_size - 3) + 1;
        }
        current_num = 1;
        ntf_value_p = (uint8_t *)malloc((spp_mtu_size-3)*sizeof(uint8_t));
        if(ntf_value_p == NULL){
            syslog(LOG_ERR, "%s malloc.2 failed\n", __func__);
            free(temp);
            lua_pushnil(L);
            lua_pushstring(L, "malloc.2 failed");
            return 2;
        }
        
        while(current_num <= total_num){
            if(current_num < total_num){
                memcpy(ntf_value_p,temp + (current_num - 1)*(spp_mtu_size-3),(spp_mtu_size-7));
                esp_ble_gatts_send_indicate(spp_gatts_if, spp_conn_id, spp_handle_table[SPP_IDX_SPP_DATA_NTY_VAL],(spp_mtu_size-3), ntf_value_p, false);
            }else if(current_num == total_num){
                memcpy(ntf_value_p,temp + (current_num - 1)*(spp_mtu_size-3),(length - (current_num - 1)*(spp_mtu_size - 3)));
                esp_ble_gatts_send_indicate(spp_gatts_if, spp_conn_id, spp_handle_table[SPP_IDX_SPP_DATA_NTY_VAL],(length - (current_num - 1)*(spp_mtu_size - 3)), ntf_value_p, false);
            }
            vTaskDelay(20 / portTICK_PERIOD_MS);
            current_num++;
        }
        free(ntf_value_p);
    }
    free(temp);

    lua_pushboolean(L, true);
    return 1;
}

static int robotito_ble_rcv (lua_State *L) {
    bool enable = lua_toboolean(L, 1);
    if (enable) {
	    luaL_checktype(L, 1, LUA_TFUNCTION);
        lua_pushvalue(L, 1);
        robotito_ble_rcv_callback = luaL_ref(L, LUA_REGISTRYINDEX);
    } else {
        if (robotito_ble_rcv_callback==LUA_REFNIL) {
            lua_pushnil(L);
            lua_pushstring(L, "no rcv callback set");
            return 2;
        }
        robotito_ble_rcv_callback = LUA_REFNIL;
    }

    lua_pushboolean(L, true);
	return 1;
}

static int robotito_ble_line (lua_State *L) {
    bool enable = lua_toboolean(L, 1);
    if (enable) {
        if (line_buff==NULL) {
            line_buff = (char*)malloc(sizeof(char)*CONFIG_ROBOTITO_BLE_LINEBUFFER);
            if(line_buff == NULL){
                syslog(LOG_ERR, "%s malloc failed\n", __func__);
                lua_pushnil(L);
                lua_pushstring(L, "malloc failed");
                return 2;
            }   
        }

	    luaL_checktype(L, 1, LUA_TFUNCTION);
        lua_pushvalue(L, 1);
        robotito_ble_line_callback = luaL_ref(L, LUA_REGISTRYINDEX);
    } else {
        if (robotito_ble_line_callback==LUA_REFNIL) {
            lua_pushnil(L);
            lua_pushstring(L, "no line callback set");
            return 2;
        }
        robotito_ble_line_callback = LUA_REFNIL;
    }

    lua_pushboolean(L, true);
	return 1;
}

static const luaL_Reg robotito_ble[] = {
    {"init", robotito_ble_init},
    {"send", robotito_ble_send},
    {"set_rcv_callback", robotito_ble_rcv},
    {"set_line_callback", robotito_ble_line},
    {NULL, NULL}
};

LUALIB_API int luaopen_robotito_ble( lua_State *L ) {
    luaL_newlib(L, robotito_ble);
    return 1;
}

MODULE_REGISTER_RAM(ROBOTITO_BLE, robotito_ble, luaopen_robotito_ble, 1);

#endif
