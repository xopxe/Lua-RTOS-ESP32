/*
 * Copyright (C) 2015 - 2020, IBEROXARXA SERVICIOS INTEGRALES, S.L.
 * Copyright (C) 2015 - 2020, Jaume Olivé Petrus (jolive@whitecatboard.org)
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
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
 * Lua RTOS, Lua rfid module *
 */
 
#include "sdkconfig.h"
#if CONFIG_LUA_RTOS_LUA_USE_RFID

#include <stdio.h>
#include <string.h>

#include <uart.h>

#include "lua.h"
#include "lualib.h"
#include "lauxlib.h"
#include "error.h"
#include "sys.h"
#include "modules.h"
#include "luartos.h"
#include <time.h>
#include <stdbool.h>


#include <drivers/gpio.h>
#include <drivers/cpu.h>
#include <drivers/uart.h>

#define RFID_UART_ID 2
#define PIO_RX 4
#define PIO_TX 17

#define BAUDS 9600 
#define DATABITS 8
#define PARITY 0
#define STOP_BITS 1




TimerHandle_t rfid_get_tag_timer;
int rfid_get_tag_callback = LUA_REFNIL;
int rfid_detected = 0;

static bool initialized = false;

char tag[10]="";
char lastTag[10]="";
char checksumTag[2]="";


int contTag=0;
int contNil=0;

int requiresNone=1;

int contReadBytes=0;

int headByteRead =0;


static int uart_exists(int id) {
    return ((id >= CPU_FIRST_UART) && (id <= CPU_LAST_UART));
}

static int rfid_init (lua_State *L) {

    if (!initialized) {
        driver_error_t *error;
        if ((error = uart_pin_map(RFID_UART_ID, PIO_RX, PIO_TX))) {
            return luaL_driver_error(L, error);
        }

        int flags = UART_FLAG_WRITE | UART_FLAG_READ;
        int buffer = luaL_optinteger(L, 6, 1024);
        // Setup
        error = uart_init(RFID_UART_ID, BAUDS, DATABITS, PARITY, STOP_BITS, flags, buffer);
        if (error) {
            return luaL_driver_error(L, error);
        }

        error = uart_setup_interrupts(RFID_UART_ID);
        if (error) {
            return luaL_driver_error(L, error);
        }

        initialized = true;
        lua_pushboolean(L, true);
        return 1;
    }
    return 1;
}

static int set_requiresNone(lua_State* L) {

	bool requires = lua_toboolean(L, 1);
    if (requires){
        requiresNone=1;
    } else {
        requiresNone=0;
    }

	return 0;
}
/*
static int luart_read( lua_State* L ) {
    int res, c;
    
    // Some integrity checks
    if (!uart_exists(2)) {
        return luaL_error(L, "UART%d does not exist", 2);
    }
    
    if (!uart_is_setup(2)) {
        return luaL_error(L, "UART%d is not setup", 2);
    }
    

    res = uart_read(2, (char *)&c, 5);
    if (res) {
        lua_pushinteger(L, c & 0x000000ff);
    } else {
        lua_pushnil(L);
    }
        
    return 1;    
}

bool verify_checksum(char *tag, char *checksumTag) {
    unsigned short calculated_checksum = 0;
    for(int i = 0; i < strlen(tag); i += 2) {
        unsigned short byte_pair = (tag[i] << 8) | tag[i+1];
        calculated_checksum ^= byte_pair;
    }

    unsigned short checksum_bytes = (checksumTag[0] << 8) | checksumTag[1];

    return calculated_checksum == checksum_bytes;
}*/

static void callback_rfid_get_tag(TimerHandle_t xTimer) {
    lua_State *TL;
	lua_State *L;
	int tref;
    int id = 2;
    int timeout, res, c;
    int status;

    if (!uart_exists(id) || !uart_is_setup(id)) {
        L = pvGetLuaState();
        TL = lua_newthread(L);
        tref = luaL_ref(L, LUA_REGISTRYINDEX);
        lua_rawgeti(L, LUA_REGISTRYINDEX, rfid_get_tag_callback);
        lua_xmove(L, TL, 1);

        lua_pushnil(TL);
        lua_pushstring(TL, "Erro, UART does not exist or is not setup");
        status = lua_pcall(TL, 2, 0, 0);
        luaL_unref(TL, LUA_REGISTRYINDEX, tref);

        if (status != LUA_OK) {
            const char *msg = lua_tostring(TL, -1);
            lua_writestringerror("error in rfid callback not exist %s\n", msg);
            lua_pop(TL, 1);		
        }   

        memset(tag, '\0', sizeof(tag));
        memset(checksumTag, '\0', sizeof(checksumTag)); 
        printf("LL: error ouart no configurado\n");


    } else {
        timeout = 0;
        res = uart_read(id, (char *)&c, timeout);

        if (rfid_detected){
            
            if(!res){

                contNil = contNil + 1;
                if (contNil == 110 ){
                    L = pvGetLuaState();
                    TL = lua_newthread(L);
                    tref = luaL_ref(L, LUA_REGISTRYINDEX);
                    lua_rawgeti(L, LUA_REGISTRYINDEX, rfid_get_tag_callback);
                    lua_xmove(L, TL, 1);

                    rfid_detected=false;
                    lua_pushstring(TL, "none");
                    status = lua_pcall(TL, 1, 0, 0);
                    
                    luaL_unref(TL, LUA_REGISTRYINDEX, tref);

                    if (status != LUA_OK) {
                        const char *msg = lua_tostring(TL, -1);
                        lua_writestringerror("error in rfid callback none %s\n", msg);
                        lua_pop(TL, 1);
                    }
                    contNil=0;
                    contTag=0;
                    printf("LL: none detected\n");

                }  
            } else {
                if (requiresNone==0){
                    rfid_detected = false;
                } else {
                    contNil = 0;
                    uart_consume(id);
                }
                

            }
            
        } else { //not rfid_detected
            if(res){
                c = c & 0x000000ff;
                printf( "%d", c );
                if ((((tag[0] == '\0') && c == 0x00000002) || headByteRead)){
                    if (c == 0x00000002){ // principio del tag
                       headByteRead = 1;
                       contReadBytes=0;
                    } else if (c == 0x00000003){ //fin del tag
                        /*printf("verificar checksum\n");
                        bool checksum = verify_checksum(tag, checksumTag);
                        printf("Your boolean variable is: %s", checksum ? "true" : "false");*/

                        if (contTag==0){
                            strncpy( lastTag, tag, sizeof(tag));
                            contTag= contTag+1;
                            memset(tag, '\0', sizeof(tag));
                            memset(checksumTag, '\0', sizeof(checksumTag));
                            printf("LL: scnd tag det\n");

                        } else {
                            if(strncmp(lastTag, tag, sizeof(tag)) == 0){
                                // si el tag es igual al anterior sumo uno al contador
                                contTag= contTag+1;
                                memset(tag, '\0', sizeof(tag));
                                memset(checksumTag, '\0', sizeof(checksumTag));
                                printf("LL: fisrt tag det\n");

                            } else {
                                // si el tag es distinto reincio los contadores a 0
                                contTag=0;
                                memset(tag, '\0', sizeof(tag));
                                memset(lastTag, '\0', sizeof(tag));
                                memset(checksumTag, '\0', sizeof(checksumTag));
                                printf("LL: tag distinto al anterior\n");


                            }
                        }    
                        headByteRead = 0;
 
                    } else { // estoy en el medio
                        char *pChar = (char*)&c;
                        if (contReadBytes>10){
                            strcat(checksumTag, pChar);
                        } else {
                            strcat(tag, pChar);
                        }
                    }
                    if (contTag==2){
                        L = pvGetLuaState();
                        TL = lua_newthread(L);
                        tref = luaL_ref(L, LUA_REGISTRYINDEX);
                        lua_rawgeti(L, LUA_REGISTRYINDEX, rfid_get_tag_callback);
                        lua_xmove(L, TL, 1);

                        lua_pushstring (TL, lastTag);
                        status = lua_pcall(TL, 1, 0, 0);
                        
                        luaL_unref(TL, LUA_REGISTRYINDEX, tref);
                        if (status != LUA_OK) {
                            const char *msg = lua_tostring(TL, -1);
                            lua_writestringerror("error in rfid callback tag %s\n", msg);
                            lua_pop(TL, 1);
                        }
                        memset(tag, '\0', sizeof(tag));
                        memset(lastTag, '\0', sizeof(lastTag));
                        memset(checksumTag, '\0', sizeof(checksumTag));
                        rfid_detected=true;
                        contTag=0;
                        contNil=0;
                        uart_consume(id);
                        printf("LL: tag detected\n");

                    }

                }else{
                    contTag=0;
                    memset(tag, '\0', sizeof(tag));
                    memset(lastTag, '\0', sizeof(tag)); 
                    memset(checksumTag, '\0', sizeof(checksumTag));

                }
               
            } else{
                contTag=0;
            } 
        }
        
    }

    
}



static int rfid_get_tag (lua_State *L) {
    bool enable = lua_toboolean(L, 1);
    if (enable) {
        luaL_checktype(L, 1, LUA_TFUNCTION);
        lua_pushvalue(L, 1);
        rfid_get_tag_callback = luaL_ref(L, LUA_REGISTRYINDEX);
        printf("LL: callback registrado\n");

    } else {
        if (rfid_get_tag_callback==LUA_REFNIL) {
            lua_pushnil(L);
            lua_pushstring(L, "no rfid get running");
            return 2;
        }
        rfid_get_tag_callback = LUA_REFNIL;
    }

    lua_pushboolean(L, true);
	return 1;
}

static int rfid_enable (lua_State *L) {
    bool enable = lua_toboolean(L, 1);
    if (enable) {
        uint32_t millis = luaL_checkinteger( L, 1 ); //segundo param (period)
        if (millis < 1) { //si me mandan periodo invalido tiro error 
            lua_pushnil(L);
            lua_pushstring(L, "invalid period");
            return 2;
        }
        //millis = 10;
            //set timer for callback
        rfid_get_tag_timer = xTimerCreate("rfid", millis / portTICK_PERIOD_MS, pdTRUE,
                (void *)rfid_get_tag_timer, callback_rfid_get_tag);
            xTimerStart(rfid_get_tag_timer, 0);
        printf("LL: rfid enable\n");

    }else {

        //delete timer
        xTimerStop(rfid_get_tag_timer, portMAX_DELAY);
	    xTimerDelete(rfid_get_tag_timer, portMAX_DELAY);
        luaL_unref(L, LUA_REGISTRYINDEX, rfid_get_tag_callback); // Elimina la referencia
        rfid_get_tag_callback = LUA_REFNIL;
        printf("LL: rfid disable, call unref\n");
                        
    }
   

    lua_pushboolean(L, true);
	return 1;
}


static const luaL_Reg rfid2[] = {
  {"init", rfid_init},
  {"set_callback", rfid_get_tag},
  {"enable", rfid_enable},
  //{"read_sensor",luart_read},
  {"requires_none",set_requiresNone},
  {NULL, NULL}
};

LUALIB_API int luaopen_rfid( lua_State *L ) {
    luaL_newlib(L, rfid2);
    return 1;
}

MODULE_REGISTER_RAM(RFID, rfid2, luaopen_rfid, 1);

#endif
