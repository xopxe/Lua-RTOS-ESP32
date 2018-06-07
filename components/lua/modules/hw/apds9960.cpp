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

#include <drivers/apds9960.h>

TimerHandle_t apds9660_get_color_timer;
int apds9660_get_color_callback=LUA_REFNIL;


SparkFun_APDS9960 sensor;

static void callback_sw_get_color(TimerHandle_t xTimer) {
	lua_State *TL;
	lua_State *L;
	int tref;

	L = pvGetLuaState();
    TL = lua_newthread(L);

    tref = luaL_ref(L, LUA_REGISTRYINDEX);

    lua_rawgeti(L, LUA_REGISTRYINDEX, apds9660_get_color_callback);
    lua_xmove(L, TL, 1);

    //push reading
    bool ok;
    uint16_t R;
    uint16_t G;
    uint16_t B;
    uint16_t A;
    ok = sensor.readRedLight(R) 
      && sensor.readGreenLight(G)
      && sensor.readBlueLight(B)
      && sensor.readAmbientLight(A);

    int status;
    if (ok) {
        lua_pushinteger(TL, R);
        lua_pushinteger(TL, G);
        lua_pushinteger(TL, B);
        lua_pushinteger(TL, A);
        status = lua_pcall(TL, 4, 0, 0);
    } else {
        lua_pushnil(TL);
        lua_pushstring(TL, "Failure reading color");
        status = lua_pcall(TL, 4, 0, 0);
    }
    luaL_unref(TL, LUA_REGISTRYINDEX, tref);

    if (status != LUA_OK) {
    	const char *msg = lua_tostring(TL, -1);
    	luaL_error(TL, msg);
    }
}

static int apds9660_init (lua_State *L) {
    lua_pushboolean(L, true);
	return 1;
}

static int apds9660_get_color_continuous (lua_State *L) {
    bool enable = lua_toboolean(L, 1);
    if (enable) {
        if (apds9660_get_color_callback!=LUA_REFNIL) {
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

        //set timer for callback
	    luaL_checktype(L, 2, LUA_TFUNCTION);
        lua_pushvalue(L, 2);
        apds9660_get_color_callback = luaL_ref(L, LUA_REGISTRYINDEX);
        apds9660_get_color_timer = xTimerCreate("apds9960color", millis / portTICK_PERIOD_MS, pdTRUE, 
            (void *)apds9660_get_color_timer, callback_sw_get_color);
        xTimerStart(apds9660_get_color_timer, 0);
    } else {
        if (apds9660_get_color_callback==LUA_REFNIL) {
            lua_pushnil(L);
            lua_pushstring(L, "no continuos get running");
            return 2;
        }

        //delete timer
        xTimerStop(apds9660_get_color_timer, portMAX_DELAY);
		xTimerDelete(apds9660_get_color_timer, portMAX_DELAY);
        apds9660_get_color_callback=LUA_REFNIL;
    }

    lua_pushboolean(L, true);
	return 1;
}

static const luaL_Reg apds9660[] = {
	{"init", apds9660_init},
	{"get_color_continuous", apds9660_get_color_continuous},
    {NULL, NULL}
};

LUALIB_API int luaopen_apds9660( lua_State *L ) {
    //luaL_register(L,"vl53l0x", vl53l0x_map);
    luaL_newlib(L, apds9660);
	return 1;
}

MODULE_REGISTER_RAM(APDS9660, apds9660, luaopen_apds9660, 1);

#ifdef __cplusplus
}
#endif

#endif
