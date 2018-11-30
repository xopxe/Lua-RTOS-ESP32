#include "sdkconfig.h"

#if CONFIG_LUA_RTOS_LUA_USE_VL53RING

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

#include <drivers/VL53L0X.h>
#include <drivers/gpio.h>

#define ADDRESS_DEFAULT 0b0101001

TimerHandle_t vl53ring_get_timer;
int vl53ring_get_callback=LUA_REFNIL;

typedef struct {
    int xshut_pin;
    VL53L0X vl53l0x;
} sensor_t;


static sensor_t *sensors;
int n_sensors;

static void callback_sw_dist(TimerHandle_t xTimer) {
	lua_State *TL;
	lua_State *L;
	int tref;

	L = pvGetLuaState();
    TL = lua_newthread(L);

    tref = luaL_ref(L, LUA_REGISTRYINDEX);

    lua_rawgeti(L, LUA_REGISTRYINDEX, vl53ring_get_callback);
    lua_xmove(L, TL, 1);

    //push reading
    for (int i=0; i<n_sensors; i++) {
        uint16_t dist = sensors[i].vl53l0x.readRangeContinuousMillimeters();
        lua_pushinteger(TL, dist);
    }

    int status = lua_pcall(TL, n_sensors, 0, 0);
    luaL_unref(TL, LUA_REGISTRYINDEX, tref);

    if (status != LUA_OK) {
		const char *msg = lua_tostring(TL, -1);
    	//luaL_error(TL, msg);
		lua_writestringerror("error in distance callback %s\n", msg);
		lua_pop(TL, 1);		
    }
}

static int lvl53ring_init (lua_State *L) {
	driver_error_t *error;

    if (lua_type(L,1)!=LUA_TTABLE) {
        lua_pushnil(L);
        lua_pushstring(L, "bad parameter, must be table");
    	return 2;
    }

    lua_len(L,1);
    n_sensors = luaL_checkinteger( L, -1 );
    lua_pop(L,1);

    printf("n_sensors=%i\r\n", n_sensors);

    sensors = new sensor_t [n_sensors];
    //int xshut_pins[n_sensors];
    int remapaddress[n_sensors];

    for (int i=1; i<=n_sensors; i++) {
        lua_rawgeti(L,1, i); // entry at stack top
        {
            if (lua_type(L,-1)!=LUA_TTABLE) {
                lua_pushnil(L);
                lua_pushstring(L, "bad entry in parameter, must be table");
            	return 2;
            }

            lua_rawgeti(L,-1,1); // first entry
            int xshut = lua_tointeger(L,-1);
            lua_pop(L,1);

            lua_rawgeti(L,-1,2); // second entry
            int newaddr = luaL_optinteger(L,-1, 0);
            if (newaddr<=0) newaddr = ADDRESS_DEFAULT + i;
            lua_pop(L,1);

            printf("xshut=%i newaddr=%i\r\n", xshut, newaddr);
            sensors[i-1].xshut_pin = xshut;
            remapaddress[i-1] = newaddr;

        }
        lua_pop(L, 1); // pop entry
    }


    //put all sensors in reset
    for (int i=0; i<n_sensors; i++) {
        int pin = sensors[i].xshut_pin;

        if ((error = gpio_pin_output(pin))) {
            lua_pushnil(L);
            lua_pushstring(L, "error setting pin to out");
            lua_pushinteger(L, pin);
        	return 3;
        }
        if ((error = gpio_pin_clr(pin))) {
            lua_pushnil(L);
            lua_pushstring(L, "error clearing pin");
            lua_pushinteger(L, pin);
        	return 3;
        }

    }
    usleep(5*1000);

    //init sensors and renumber
    for (int i=0; i<n_sensors; i++) {
        int pin = sensors[i].xshut_pin;

        if ((error = gpio_pin_set(pin))) {
            lua_pushnil(L);
            lua_pushstring(L, "error setting pin");
            lua_pushinteger(L, pin);
        	return 3;
        }

        usleep(5*1000);
        bool ok = sensors[i].vl53l0x.init(true);
        //printf("done: bool ok = sensors[i].vl53l0x.init();");
        if (!ok) {
            /*lua_pushnil(L);
            lua_pushstring(L, "internal sensor failure");
            return 2;*/
            printf("ring: warning: internal sensor failure\r\n");
        }

        //printf("i2c remapping check %d renumber to %d\r\n", sensors[i].vl53l0x.getAddress(), remapaddress[i] );
        if (remapaddress[i]>0 && remapaddress[i]!=sensors[i].vl53l0x.getAddress()) {
            sensors[i].vl53l0x.setAddress(remapaddress[i]);
            printf("ring: i2c for vl53l0x %d renumbered to %d\r\n", i, remapaddress[i] );
            //printf("ring: i2c vl53l0x %d measure %d\r\n", i, sensors[i].vl53l0x.readRangeSingleMillimeters());
        }

    }

    lua_pushboolean(L, true);
	return 1;
}

static int lvl53ring_release (lua_State *L) {
    delete sensors;
    sensors = NULL;
    lua_pushboolean(L, true);
	return 1;

}

static int lvl53ring_test (lua_State *L) {
    VL53L0X *vl53l0x = &(sensors[0].vl53l0x);
    uint16_t val;

    val = vl53l0x->readRangeSingleMillimeters();

    if (vl53l0x->timeoutOccurred()) {
        lua_pushinteger(L, val);
        lua_pushstring(L, "timeout");
        return 2;
    }

    lua_pushinteger(L, val);

	return 1;
}

static int lvl53ring_set_timeout (lua_State *L) {
    uint16_t timeout = luaL_checkinteger( L, 1 );

    for (int i=0; i<n_sensors; i++) {
        sensors[i].vl53l0x.setTimeout(timeout);
    }

    lua_pushboolean(L, true);
	return 1;
}

static int lvl53ring_set_measurement_timing_budget (lua_State *L) {
    uint16_t us = luaL_checkinteger( L, 1 );

    for (int i=0; i<n_sensors; i++) {
        sensors[i].vl53l0x.setMeasurementTimingBudget(us);
    }

    lua_pushboolean(L, true);
	return 1;
}

static int lvl53ring_set_measurement_timing_budget_sensor (lua_State *L) {
    uint16_t i = luaL_checkinteger( L, 1 );
    uint16_t us = luaL_checkinteger( L, 2 );

    sensors[i].vl53l0x.setMeasurementTimingBudget(us);

    lua_pushboolean(L, true);
	return 1;
}

static int lvl53ring_get_measurement_timing_budget_sensor (lua_State *L) {
    uint16_t i = luaL_checkinteger( L, 1 );

    uint32_t budget = sensors[i].vl53l0x.getMeasurementTimingBudget();

    lua_pushinteger(L, budget);
	return 1;
}

static int lvl53ring_set_signal_rate_limit (lua_State *L) {
    float rate = luaL_checknumber( L, 1 );

    for (int i=0; i<n_sensors; i++) {
        sensors[i].vl53l0x.setSignalRateLimit(rate);
    }

    lua_pushboolean(L, true);
	return 1;
}

static int lvl53ring_set_signal_rate_limit_sensor (lua_State *L) {
    uint16_t i = luaL_checkinteger( L, 1 );
    uint16_t rate = luaL_checknumber( L, 2 );

    sensors[i].vl53l0x.setSignalRateLimit(rate);

    lua_pushboolean(L, true);
	return 1;
}

static int lvl53ring_get_signal_rate_limit_sensor (lua_State *L) {
    uint16_t i = luaL_checkinteger( L, 1 );

    float rate = sensors[i].vl53l0x.getSignalRateLimit();

    lua_pushnumber(L, rate);
	return 1;
}

static int lvl53ring_get_continuous (lua_State *L) {
    bool enable = lua_toboolean(L, 1);
    if (enable) {
        if (vl53ring_get_callback!=LUA_REFNIL) {
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

        /*set sensor mode
        for (int i=0; i<n_sensors; i++) {
            sensors[i].vl53l0x.startContinuous(millis);
            usleep(1000*millis / n_sensors); // try to desynchronize readings to avoid cross-talk
        }*/

        int inter_delay = 1000*millis / n_sensors;
        sensors[0].vl53l0x.startContinuous(millis);
        usleep(inter_delay);
        sensors[3].vl53l0x.startContinuous(millis);
        usleep(inter_delay);
        sensors[1].vl53l0x.startContinuous(millis);
        usleep(inter_delay);
        sensors[4].vl53l0x.startContinuous(millis);
        usleep(inter_delay);
        sensors[2].vl53l0x.startContinuous(millis);
        usleep(inter_delay);
        sensors[5].vl53l0x.startContinuous(millis);
        usleep(inter_delay);


        //set timer for callback
	    luaL_checktype(L, 2, LUA_TFUNCTION);
        lua_pushvalue(L, 2);
        vl53ring_get_callback = luaL_ref(L, LUA_REGISTRYINDEX);
        vl53ring_get_timer = xTimerCreate("vl53ring", millis / portTICK_PERIOD_MS, pdTRUE, (void *)vl53ring_get_timer, callback_sw_dist);
        xTimerStart(vl53ring_get_timer, 0);
    } else {
        if (vl53ring_get_callback==LUA_REFNIL) {
            lua_pushnil(L);
            lua_pushstring(L, "no continuos get running");
            return 2;
        }

        //reset sensor mode
        for (int i=0; i<n_sensors; i++) {
            sensors[i].vl53l0x.stopContinuous();
        }

        //delete timer
        xTimerStop(vl53ring_get_timer, portMAX_DELAY);
		xTimerDelete(vl53ring_get_timer, portMAX_DELAY);
        vl53ring_get_callback=LUA_REFNIL;
    }

    lua_pushboolean(L, true);
	return 1;
}

static int lvl53ring_get (lua_State *L) {
    if (vl53ring_get_callback!=LUA_REFNIL) {
        lua_pushnil(L);
        lua_pushstring(L, "continuos get running");
        return 2;
    }

    uint32_t sensor_number = luaL_checkinteger( L, 1 );
    if (sensor_number < 1 || sensor_number>n_sensors) {
        lua_pushnil(L);
        lua_pushstring(L, "invalid sensor_number");
        return 2;
    }

    uint16_t reading = sensors[sensor_number-1].vl53l0x.readRangeSingleMillimeters();

    lua_pushnumber(L, reading);
	return 1;
}


static const luaL_Reg vl53ring[] = {
//	{"attach", lvl53l0x_attach},
//	{"detach", lvl53l0x_detach},
    {"init", lvl53ring_init},
    {"release", lvl53ring_release},
    {"test", lvl53ring_test},
    {"set_timeout", lvl53ring_set_timeout},
    {"set_measurement_timing_budget", lvl53ring_set_measurement_timing_budget},
    {"set_measurement_timing_budget_sensor", lvl53ring_set_measurement_timing_budget_sensor},
    {"get_measurement_timing_budget_sensor", lvl53ring_get_measurement_timing_budget_sensor},
    {"set_signal_rate_limit", lvl53ring_set_signal_rate_limit},
    {"set_signal_rate_limit_sensor", lvl53ring_set_signal_rate_limit_sensor},
    {"get_signal_rate_limit_sensor", lvl53ring_get_signal_rate_limit_sensor},
    {"get_continuous", lvl53ring_get_continuous},
    {"get", lvl53ring_get},
    {NULL, NULL}
};

LUALIB_API int luaopen_vl53ring( lua_State *L ) {
    //luaL_register(L,"vl53l0x", vl53l0x_map);
    luaL_newlib(L, vl53ring);
	return 1;
}

MODULE_REGISTER_RAM(VL53RING, vl53ring, luaopen_vl53ring, 1);


/*
static const LUA_REG_TYPE vl53l0x_inst_map[] = {
	{ LSTRKEY( "detach" ),			LFUNCVAL( lvl53l0x_detach  ) },
	{ LSTRKEY( "read_range_single_millimeters" ), LFUNCVAL( lvl53l0x_readRangeSingleMillimeters ) },
    { LSTRKEY( "__metatable" ),	    LROVAL  ( vl53l0x_inst_map ) },
	{ LSTRKEY( "__index"     ),     LROVAL  ( vl53l0x_inst_map ) },
    { LSTRKEY( "__gc" ),	 	    LFUNCVAL( lvl53l0x_gc 	   ) },
    { LNILKEY, LNILVAL }
};

static const LUA_REG_TYPE vl53l0x_map[] = {
	{ LSTRKEY( "attach" ),			LFUNCVAL( lvl53l0x_attach ) },
    { LNILKEY, LNILVAL }
};

LUALIB_API int luaopen_vl53l0x( lua_State *L ) {
    luaL_newmetarotable(L,"vl53l0x.ins", (void *)vl53l0x_inst_map);
	return 0;
}

MODULE_REGISTER_ROM(VL53L0X, vl53l0x, vl53l0x_map, luaopen_vl53l0x, 1);
*/

#ifdef __cplusplus
}
#endif

#endif
