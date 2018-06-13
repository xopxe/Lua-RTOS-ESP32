#include "sdkconfig.h"

#if CONFIG_LUA_RTOS_LUA_USE_APDS9960

#ifdef __cplusplus
extern "C"{
#endif

#include "freertos/FreeRTOS.h"
#include "freertos/adds.h"
#include "freertos/timers.h"

#include "modules.h"
//#include "error.h"

#ifdef __cplusplus
  #include "lua.hpp"
#else
  #include "lua.h"
  #include "lualib.h"
  #include "lauxlib.h"
#endif

#include "apds9960.h"
#include <drivers/apds9960.h>

TimerHandle_t apds9960_get_rgb_timer;
int apds9960_get_rgb_callback=LUA_REFNIL;

TimerHandle_t apds9960_get_colorchange_timer;
int apds9960_get_colorchange_callback=LUA_REFNIL;

bool hsv_mode = false;
int current_color_i = -1;
int saturation_threshold = 0;
int value_threshold = 0;

SparkFun_APDS9960 sensor;

static void RGB2HSV(struct RGB_set RGB, struct HSV_set &HSV);

static void callback_sw_get_rgb(TimerHandle_t xTimer) {
	lua_State *TL;
	lua_State *L;
	int tref;

	L = pvGetLuaState();
    TL = lua_newthread(L);

    tref = luaL_ref(L, LUA_REGISTRYINDEX);

    lua_rawgeti(L, LUA_REGISTRYINDEX, apds9960_get_rgb_callback);
    lua_xmove(L, TL, 1);

    //push reading
    uint16_t R;
    uint16_t G;
    uint16_t B;
    uint16_t A;

    bool ok = sensor.readColors(R, G, B, A);

    int status;
    if (ok) {
        lua_pushinteger(TL, R);
        lua_pushinteger(TL, G);
        lua_pushinteger(TL, B);
        lua_pushinteger(TL, A);
        if (hsv_mode) {
            status = lua_pcall(TL, 4, 0, 0);
        } else {
            HSV_set hsv;
            RGB_set rgb;
            rgb.r = R; //R<<8;
            rgb.g = G; //G<<8;
            rgb.b = B; //B<<8;
            RGB2HSV(rgb , hsv);
            lua_pushinteger(TL, hsv.h);
            lua_pushinteger(TL, hsv.s);
            lua_pushinteger(TL, hsv.v);

            int color_i = -1;
            //find color index in color_ranges[]
            for (int i=0; i<6; i++) {
                if (hsv.v>=color_ranges[i].min && hsv.v<=color_ranges[i].max) {
                    color_i=i;
                    break;
                }
            }
            lua_pushstring(TL, color_ranges[color_i].name);

            status = lua_pcall(TL, 8, 0, 0);
        }
    } else {
        lua_pushnil(TL);
        lua_pushstring(TL, "Failure reading color");
        status = lua_pcall(TL, 2, 0, 0);
    }
    luaL_unref(TL, LUA_REGISTRYINDEX, tref);

    if (status != LUA_OK) {
    	const char *msg = lua_tostring(TL, -1);
    	luaL_error(TL, msg);
    }
}

/*
static void prepare_thread (lua_State *TL, lua_State *L, int &tref) {
	L = pvGetLuaState();
    TL = lua_newthread(L);

    tref = luaL_ref(L, LUA_REGISTRYINDEX);

    lua_rawgeti(L, LUA_REGISTRYINDEX, apds9960_get_colorchange_callback);
    lua_xmove(L, TL, 1);
}
*/

static void callback_sw_get_colorchange(TimerHandle_t xTimer) {
	lua_State *TL;
	lua_State *L;
	int tref;

    /*    
    L = pvGetLuaState();
    TL = lua_newthread(L);

    tref = luaL_ref(L, LUA_REGISTRYINDEX);

    lua_rawgeti(L, LUA_REGISTRYINDEX, apds9960_get_colorchange_callback);
    lua_xmove(L, TL, 1);
    */

    uint16_t R;
    uint16_t G;
    uint16_t B;
    uint16_t A;
    bool ok = sensor.readColors(R, G, B, A);

    int status;
    if (ok) {
        HSV_set hsv;
        RGB_set rgb;
        rgb.r = R; //R<<8;
        rgb.g = G; //G<<8;
        rgb.b = B; //B<<8;
        RGB2HSV(rgb , hsv);
        
        if (hsv.s<saturation_threshold || hsv.v<value_threshold) {
            return;
        }
        
        int color_i = -1;
        //find color index in color_ranges[]
        for (int i=0; i<6; i++) {
            if (hsv.v>=color_ranges[i].min && hsv.v<=color_ranges[i].max) {
                color_i=i;
                break;
            }
        }
        
        if (color_i!=current_color_i) {
            current_color_i = color_i;

            //prepare thread
            L = pvGetLuaState();
            TL = lua_newthread(L);
            tref = luaL_ref(L, LUA_REGISTRYINDEX);
            lua_rawgeti(L, LUA_REGISTRYINDEX, apds9960_get_colorchange_callback);
            lua_xmove(L, TL, 1);

            lua_pushstring(TL, color_ranges[color_i].name);
            lua_pushinteger(TL, hsv.s);
            lua_pushinteger(TL, hsv.v);
            status = lua_pcall(TL, 3, 0, 0);
            luaL_unref(TL, LUA_REGISTRYINDEX, tref);

        } else {
            return; //no changes
        }
        
    } else {
        //prepare thread
        L = pvGetLuaState();
        TL = lua_newthread(L);
        tref = luaL_ref(L, LUA_REGISTRYINDEX);
        lua_rawgeti(L, LUA_REGISTRYINDEX, apds9960_get_colorchange_callback);
        lua_xmove(L, TL, 1);

        lua_pushnil(TL);
        lua_pushstring(TL, "Failure reading color");
        status = lua_pcall(TL, 2, 0, 0);
        luaL_unref(TL, LUA_REGISTRYINDEX, tref);
    }

    if (status != LUA_OK) {
    	const char *msg = lua_tostring(TL, -1);
    	luaL_error(TL, msg);
    }
}


static int apds9960_init (lua_State *L) {
    bool ok;
    ok = sensor.init();
    if (!ok) {
        lua_pushnil(L);
        lua_pushstring(L, "color sensor initalization failed");
        return 2;
    }
    ok = sensor.enablePower();
    if (!ok) {
        lua_pushnil(L);
        lua_pushstring(L, "color sensor power up failed");
        return 2;
    }
    ok = sensor.enableLightSensor(false);
    if (!ok) {
        lua_pushnil(L);
        lua_pushstring(L, "color sensor enable failed");
        return 2;
    }
    lua_pushboolean(L, true);
    return 1;
}

static int apds9960_get_rgb_continuous (lua_State *L) {
    bool enable = lua_toboolean(L, 1);
    if (enable) {
        if (apds9960_get_rgb_callback!=LUA_REFNIL) {
            lua_pushnil(L);
            lua_pushstring(L, "continuos get already running");
            return 2;
        }

        uint32_t millis = luaL_checkinteger( L, 1 );
	    if (millis < 1) {
            lua_pushnil(L);
            lua_pushstring(L, "invalid period");
            return 2;
	    }
	    hsv_mode = lua_toboolean(L, 3);

        //set timer for callback
	    luaL_checktype(L, 2, LUA_TFUNCTION);
        lua_pushvalue(L, 2);
        apds9960_get_rgb_callback = luaL_ref(L, LUA_REGISTRYINDEX);
        apds9960_get_rgb_timer = xTimerCreate("apds9960rgb", millis / portTICK_PERIOD_MS, pdTRUE, 
            (void *)apds9960_get_rgb_timer, callback_sw_get_rgb);
        xTimerStart(apds9960_get_rgb_timer, 0);
    } else {
        if (apds9960_get_rgb_callback==LUA_REFNIL) {
            lua_pushnil(L);
            lua_pushstring(L, "no continuos get running");
            return 2;
        }

        //delete timer
        xTimerStop(apds9960_get_rgb_timer, portMAX_DELAY);
		xTimerDelete(apds9960_get_rgb_timer, portMAX_DELAY);
        apds9960_get_rgb_callback=LUA_REFNIL;
    }

    lua_pushboolean(L, true);
	return 1;
}

static int apds9960_get_rgb (lua_State *L) {
    uint16_t R;
    uint16_t G;
    uint16_t B;
    uint16_t A;
    bool ok = sensor.readColors(R, G, B, A);
    if (ok) {
        lua_pushinteger(L, R);
        lua_pushinteger(L, G);
        lua_pushinteger(L, B);
        lua_pushinteger(L, A);
        return 4;
    } else {
        lua_pushnil(L);
        lua_pushstring(L, "Failure reading color");
        return 2;
    }
}


static int apds9960_get_ambient (lua_State *L) {
    uint16_t A;
    bool ok = sensor.readAmbientLight(A);
    if (ok) {
        lua_pushinteger(L, A);
        return 1;
    } else {
        lua_pushnil(L);
        lua_pushstring(L, "Failure reading ambient light");
        return 2;
    }
}


static int apds9960_get_colorchange (lua_State *L) {
    bool enable = lua_toboolean(L, 1);
    saturation_threshold = luaL_optinteger( L, 2, 0 );
    value_threshold = luaL_optinteger( L, 3, 0 );
    if (enable) {
        if (apds9960_get_colorchange_callback!=LUA_REFNIL) {
            lua_pushnil(L);
            lua_pushstring(L, "colorchange get already running");
            return 2;
        }

        uint32_t millis = luaL_checkinteger( L, 1 );
	    if (millis < 1) {
            lua_pushnil(L);
            lua_pushstring(L, "invalid period");
            return 2;
	    }
	    hsv_mode = lua_toboolean(L, 3);

        //set timer for callback
	    luaL_checktype(L, 2, LUA_TFUNCTION);
        lua_pushvalue(L, 2);
        apds9960_get_colorchange_callback = luaL_ref(L, LUA_REGISTRYINDEX);
        apds9960_get_colorchange_timer = xTimerCreate("apds9960colorchange", millis / portTICK_PERIOD_MS, pdTRUE, 
            (void *)apds9960_get_colorchange_timer, callback_sw_get_colorchange);
        xTimerStart(apds9960_get_colorchange_timer, 0);
    } else {
        if (apds9960_get_colorchange_callback==LUA_REFNIL) {
            lua_pushnil(L);
            lua_pushstring(L, "no continuos get running");
            return 2;
        }

        //delete timer
        xTimerStop(apds9960_get_colorchange_timer, portMAX_DELAY);
		xTimerDelete(apds9960_get_colorchange_timer, portMAX_DELAY);
        apds9960_get_colorchange_callback=LUA_REFNIL;
    }

    lua_pushboolean(L, true);
	return 1;
}


/*******************************************************************************
 * Function RGB2HSV
 * Description: Converts an RGB color value into its equivalen in the HSV color space.
 * Copyright 2010 by George Ruinelli
 * The code I used as a source is from http://www.cs.rit.edu/~ncs/color/t_convert.html
 * Parameters:
 *   1. struct with RGB color (source)
 *   2. pointer to struct HSV color (target)
 * Notes:
 *   - r, g, b values are from 0..255
 *   - h = [0,360], s = [0,255], v = [0,255]
 *   - NB: if s == 0, then h = 0 (undefined)
 ******************************************************************************/
static void RGB2HSV(struct RGB_set RGB, struct HSV_set &HSV){
 unsigned char min, max, delta;
 
 if(RGB.r<RGB.g)min=RGB.r; else min=RGB.g;
 if(RGB.b<min)min=RGB.b;
 
 if(RGB.r>RGB.g)max=RGB.r; else max=RGB.g;
 if(RGB.b>max)max=RGB.b;
 
 HSV.v = max;                // v, 0..255
 
 delta = max - min;                      // 0..255, < v
 
 if( max != 0 )
 HSV.s = (int)(delta)*255 / max;        // s, 0..255
 else {
 // r = g = b = 0        // s = 0, v is undefined
 HSV.s = 0;
 HSV.h = 0;
 return;
 }
 
 if( RGB.r == max )
 HSV.h = (RGB.g - RGB.b)*60/delta;        // between yellow & magenta
 else if( RGB.g == max )
 HSV.h = 120 + (RGB.b - RGB.r)*60/delta;    // between cyan & yellow
 else
 HSV.h = 240 + (RGB.r - RGB.g)*60/delta;    // between magenta & cyan
 
 if( HSV.h < 0 )
 HSV.h += 360;
}

static const luaL_Reg apds9960[] = {
	{"init", apds9960_init},
	{"get_rgb", apds9960_get_rgb},
	{"get_ambient", apds9960_get_ambient},
	{"get_color_continuous", apds9960_get_rgb_continuous},
	{"get_color_change", apds9960_get_colorchange},
    {NULL, NULL}
};

LUALIB_API int luaopen_apds9960( lua_State *L ) {
    //luaL_register(L,"vl53l0x", vl53l0x_map);
    luaL_newlib(L, apds9960);
	return 1;
}

MODULE_REGISTER_RAM(APDS9960, apds9960, luaopen_apds9960, 1);

#ifdef __cplusplus
}
#endif

#endif
