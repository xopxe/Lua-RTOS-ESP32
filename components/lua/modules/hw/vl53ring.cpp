#include "sdkconfig.h"

#if CONFIG_LUA_RTOS_LUA_USE_VL53RING

#ifdef __cplusplus
extern "C"{
#endif

#include "freertos/FreeRTOS.h"
#include "freertos/adds.h"

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

#define NSENSORS 1
#define XSHUT_PINS {19,13,14,15,16,17}
#define REMAPADDRESS {0, 0x11,0x12,0x13,0x14,0x15}


typedef struct {
    int xshut_pin;
    VL53L0X vl53l0x;
} sensor_t;


static sensor_t sensors[NSENSORS];

//static int process_i2c_error(lua_State *L, VL53L0X &vl53l0x) {
//}

static int lvl53ring_init (lua_State *L) {
	driver_error_t *error;

    int xshut_pins[] = XSHUT_PINS;
    int remapaddress[] = REMAPADDRESS;

    //put all sensors in rest
    for (int i=0; i<NSENSORS; i++) {
        int pin = xshut_pins[i];
        sensors[i].xshut_pin = pin;

        if ((error = gpio_pin_output(pin))) {
            lua_pushnil(L);
            lua_pushstring(L, "error setting pin to out");
            lua_pushinteger(L, pin);
        	return 3;
        }
        if ((error = gpio_pin_set(pin))) {
            lua_pushnil(L);
            lua_pushstring(L, "error setting pin");
            lua_pushinteger(L, pin);
        	return 3;
        }
        
    }


    //init sensors and renumber        
    for (int i=0; i<NSENSORS; i++) {
        int pin = xshut_pins[i];

        if ((error = gpio_pin_clr(pin))) {
            lua_pushnil(L);
            lua_pushstring(L, "error clearing pin");
            lua_pushinteger(L, pin);
        	return 3;
        }
        
        bool ok = sensors[i].vl53l0x.init();
        printf("done: bool ok = sensors[i].vl53l0x.init();");
        if (~ok) { 
            lua_pushnil(L);
            lua_pushstring(L, "internal sensor failure");
            return 2;
        }

        if (remapaddress[i]>0 && remapaddress[i]!=sensors[i].vl53l0x.getAddress()) {
            sensors[i].vl53l0x.setAddress(remapaddress[i]);
        }
        
    }

    lua_pushboolean(L, true);
	return 1;
}

static int lvl53ring_read (lua_State *L) {
/*
	driver_error_t *error;
    uint16_t val;

    val = vl53l0x->readRangeSingleMillimeters();

    if (vl53l0x->timeoutOccurred()) { 
        lua_pushinteger(L, val);
        lua_pushstring(L, "timeout");
        return 2;
    }

    lua_pushinteger(L, val);
    */
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

    for (int i=0; i<NSENSORS; i++) {
        sensors[i].vl53l0x.setTimeout(timeout);
    }

    lua_pushboolean(L, true);
	return 1;
}

static int lvl53ring_set_measurement_timing_budget (lua_State *L) {
    uint16_t us = luaL_checkinteger( L, 1 );

    for (int i=0; i<NSENSORS; i++) {
        sensors[i].vl53l0x.setMeasurementTimingBudget(us);
    }

    lua_pushboolean(L, true);
	return 1;
}

static int lvl53ring_set_continuous_mode (lua_State *L) {
    bool enable =  lua_toboolean( L, 1 );

    if (enable) {
        uint16_t ms = luaL_checkinteger( L, 2 );
        for (int i=0; i<NSENSORS; i++) {
            sensors[i].vl53l0x.startContinuous(ms);
        }
    } else {
        for (int i=0; i<NSENSORS; i++) {
            sensors[i].vl53l0x.stopContinuous();
        }
    }

    lua_pushboolean(L, true);
	return 1;
}

static int lvl53ring_get_continuous_distance (lua_State *L) {
    for (int i=0; i<NSENSORS; i++) {
        uint16_t dist = sensors[i].vl53l0x.readRangeContinuousMillimeters();
        lua_pushinteger(L, dist);
    }
	return NSENSORS;
}


static const luaL_Reg vl53ring[] = {
//	{"attach", lvl53l0x_attach},
//	{"detach", lvl53l0x_detach},
	{"init", lvl53ring_init},
	{"read", lvl53ring_read},
	{"test", lvl53ring_test},
	{"set_timeout", lvl53ring_set_timeout},
	{"set_measurement_timing_budget", lvl53ring_set_measurement_timing_budget},
	{"set_continuous_mode", lvl53ring_set_continuous_mode},
	{"get_continuous_reading", lvl53ring_get_continuous_distance},
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
