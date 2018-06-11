#include "sdkconfig.h"

#if CONFIG_LUA_RTOS_LUA_USE_VL53L0X


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

typedef struct {
    int xshut_pin;
    VL53L0X *vl53l0x;
} vl53l0x_userdata;

static int lvl53l0x_attach( lua_State* L ) {
    int xshut_pin;

	//driver_error_t *error;

    xshut_pin = luaL_checkinteger( L, 1 );
    //res = luaL_optinteger( L, 3, 0 );

    vl53l0x_userdata *udata = (vl53l0x_userdata *)lua_newuserdata(L, sizeof(vl53l0x_userdata));
    if (!udata) return 0;
    
    udata->xshut_pin = xshut_pin;
    udata->vl53l0x = new VL53L0X();
    
    luaL_getmetatable(L, "vl53l0x.ins");
    lua_setmetatable(L, -2);

    return 1;
}

//static int process_i2c_error(lua_State *L, VL53L0X &vl53l0x) {
//}

static int lvl53l0x_init (lua_State *L) {
	vl53l0x_userdata *userdata = (vl53l0x_userdata *)luaL_checkudata(L, 1, "vl53l0x.ins");
    VL53L0X *vl53l0x = userdata->vl53l0x;


    bool ok = vl53l0x->init();

    if (!ok) { 
        lua_pushnil(L);
        lua_pushstring(L, "internal failure");
        return 2;
    }

    lua_pushboolean(L, true);
	return 1;
}

static int lvl53l0x_readRangeSingleMillimeters (lua_State *L) {
	vl53l0x_userdata *userdata = (vl53l0x_userdata *)luaL_checkudata(L, 1, "vl53l0x.ins");
    uint16_t val;
    VL53L0X *vl53l0x = userdata->vl53l0x;


    val = vl53l0x->readRangeSingleMillimeters();

    if (vl53l0x->timeoutOccurred()) { 
        lua_pushinteger(L, val);
        lua_pushstring(L, "timeout");
        return 2;
    }

    lua_pushinteger(L, val);
	return 1;
}

static int lvl53l0x_detach (lua_State *L) {
	vl53l0x_userdata *userdata = (vl53l0x_userdata *)luaL_checkudata(L, 1, "vl53l0x.ins");

    delete userdata->vl53l0x;

	return 0;
}
/*
static int lvl53l0x_gc (lua_State *L) {
	lvl53l0x_detach(L);
	return 0;
}
*/

static const luaL_Reg vl53l0x[] = {
	{"attach", lvl53l0x_attach},
	{"detach", lvl53l0x_detach},
	{"init", lvl53l0x_init},
	{"read_range_single_millimeters", lvl53l0x_readRangeSingleMillimeters},
    {NULL, NULL}
};

LUALIB_API int luaopen_vl53l0x( lua_State *L ) {
    //luaL_register(L,"vl53l0x", vl53l0x_map);
    luaL_newlib(L, vl53l0x);
	return 1;
}

MODULE_REGISTER_RAM(VL53L0X, vl53l0x, luaopen_vl53l0x, 1);

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
