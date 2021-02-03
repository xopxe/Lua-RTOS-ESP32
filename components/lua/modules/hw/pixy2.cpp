#include "sdkconfig.h"

#if CONFIG_LUA_RTOS_LUA_USE_PIXY2

#ifdef __cplusplus
extern "C" {
#endif

#include <stdio.h>
#include <string.h>
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

#include "drivers/TPixy2.h"
#include "drivers/Pixy2CCC.h"
#include "drivers/Pixy2Line.h"

static uint8_t stdio;

TPixy2 pixy;
Pixy2CCC p2ccc(&pixy);
Pixy2Line p2line(&pixy);

uint16_t cb_ccc_sigmap = CCC_SIG_ALL;
uint16_t cb_ccc_maxBlocks = 0xFF;

uint16_t cb_lines_features = LINE_VECTOR | LINE_INTERSECTION | LINE_BARCODE;


int p2ccc_blocks_callback = LUA_REFNIL;
TimerHandle_t p2ccc_blocks_timer;

int p2line_lines_callback = LUA_REFNIL;
TimerHandle_t p2line_feature_timer;

static int p2_init(lua_State *L) {
	int8_t ret = pixy.init();
	if (ret == PIXY_RESULT_OK) {
		lua_pushboolean(L, true);
		return 1;
	} else {
		lua_pushnil(L);
		lua_pushinteger(L, ret);
		return 2;
	}
}

static int p2ccc_set_callback_blocks (lua_State *L) {
    bool enable = lua_toboolean(L, 1);
    if (enable) {
	    luaL_checktype(L, 1, LUA_TFUNCTION);
        lua_pushvalue(L, 1);
        p2ccc_blocks_callback = luaL_ref(L, LUA_REGISTRYINDEX);
    } else {
        if (p2ccc_blocks_callback==LUA_REFNIL) {
            lua_pushnil(L);
            lua_pushstring(L, "no continuos get running");
            return 2;
        }
        p2ccc_blocks_callback = LUA_REFNIL;
    }

    lua_pushboolean(L, true);
	return 1;
}

static int p2line_set_callback_lines (lua_State *L) {
    bool enable = lua_toboolean(L, 1);
    if (enable) {
	    luaL_checktype(L, 1, LUA_TFUNCTION);
        lua_pushvalue(L, 1);
        p2line_lines_callback = luaL_ref(L, LUA_REGISTRYINDEX);
    } else {
        if (p2line_lines_callback==LUA_REFNIL) {
            lua_pushnil(L);
            lua_pushstring(L, "no continuos get running");
            return 2;
        }
        p2line_lines_callback = LUA_REFNIL;
    }

    lua_pushboolean(L, true);
	return 1;
}

static void callback_ccc_get_blocks(TimerHandle_t xTimer) {
	lua_State *TL;
	lua_State *L;
	int tref;

    // Set standards streams
    if (!stdio) {
        __getreent()->_stdin  = _GLOBAL_REENT->_stdin;
        __getreent()->_stdout = _GLOBAL_REENT->_stdout;
        __getreent()->_stderr = _GLOBAL_REENT->_stderr;

        // Work-around newlib is not compiled with HAVE_BLKSIZE flag
        setvbuf(_GLOBAL_REENT->_stdin , NULL, _IONBF, 0);
        setvbuf(_GLOBAL_REENT->_stdout, NULL, _IONBF, 0);
        setvbuf(_GLOBAL_REENT->_stderr, NULL, _IONBF, 0);

        stdio = 1;
    }

	int8_t ret = p2ccc.getBlocks(true, cb_ccc_sigmap, cb_ccc_maxBlocks);

    int status;
    if (ret >=0) {

        ////////////
        if (p2ccc_blocks_callback!=LUA_REFNIL) {
            L = pvGetLuaState();
            TL = lua_newthread(L);

            tref = luaL_ref(L, LUA_REGISTRYINDEX);

            lua_rawgeti(L, LUA_REGISTRYINDEX, p2ccc_blocks_callback);
            lua_xmove(L, TL, 1);

            lua_createtable(TL, p2ccc.numBlocks, 0);
        	for (uint8_t i = 0; i < p2ccc.numBlocks; i++) {
        		Block block = p2ccc.blocks[i];
        		//block.print();

        		lua_pushinteger(TL, i+1);
        	    lua_createtable(TL, 0, 8);

        	    lua_pushinteger(TL, block.m_signature);
        	    lua_setfield(TL, -2, "signature");
        	    lua_pushinteger(TL, block.m_x);
        	    lua_setfield(TL, -2, "x");
        	    lua_pushinteger(TL, block.m_y);
        	    lua_setfield(TL, -2, "y");
        	    lua_pushinteger(TL, block.m_width);
        	    lua_setfield(TL, -2, "width");
        	    lua_pushinteger(TL, block.m_height);
        	    lua_setfield(TL, -2, "height");
        	    lua_pushinteger(TL, block.m_angle);
        	    lua_setfield(TL, -2, "angle");
        	    lua_pushinteger(TL, block.m_index);
        	    lua_setfield(TL, -2, "index");
        	    lua_pushinteger(TL, block.m_age);
        	    lua_setfield(TL, -2, "age");

        	    lua_settable(TL, -3);
        	}
            status = lua_pcall(TL, 1, 0, 0);
            luaL_unref(TL, LUA_REGISTRYINDEX, tref);
            if (status != LUA_OK) {
		        const char *msg = lua_tostring(TL, -1);
		        lua_writestringerror("error in ccc.get_blocks callback: %s\n", msg);
		        lua_pop(TL, 1);
                //luaL_error(TL, msg);
            }

        }

    } else {
    	printf("Error in ccc.get_blocks(): %i\n", ret);
    }
}

static void callback_line_get_features(TimerHandle_t xTimer) {
	lua_State *TL;
	lua_State *L;
	int tref;

    // Set standards streams
    if (!stdio) {
        __getreent()->_stdin  = _GLOBAL_REENT->_stdin;
        __getreent()->_stdout = _GLOBAL_REENT->_stdout;
        __getreent()->_stderr = _GLOBAL_REENT->_stderr;

        // Work-around newlib is not compiled with HAVE_BLKSIZE flag
        setvbuf(_GLOBAL_REENT->_stdin , NULL, _IONBF, 0);
        setvbuf(_GLOBAL_REENT->_stdout, NULL, _IONBF, 0);
        setvbuf(_GLOBAL_REENT->_stderr, NULL, _IONBF, 0);

        stdio = 1;
    }

    // single vector mode
	int8_t ret = p2line.getMainFeatures(cb_lines_features, true);

    int status;
    if (ret >=0) {
        if (p2line_lines_callback!=LUA_REFNIL) {
            L = pvGetLuaState();
            TL = lua_newthread(L);

            tref = luaL_ref(L, LUA_REGISTRYINDEX);

            lua_rawgeti(L, LUA_REGISTRYINDEX, p2line_lines_callback);
            lua_xmove(L, TL, 1);

            if (cb_lines_features & LINE_VECTOR) {
				/*lua_createtable(TL, p2line.numVectors, 0);
				for (uint8_t i = 0; i < p2line.numVectors; i++) {
					Vector vector = p2line.vectors[i];
					//vector.print();

					lua_pushinteger(TL, i+1);
					lua_createtable(TL, 0, 6);
					lua_pushinteger(TL, vector.m_x0);
					lua_setfield(TL, -2, "x0");
					lua_pushinteger(TL, vector.m_y0);
					lua_setfield(TL, -2, "y0");
					lua_pushinteger(TL, vector.m_x1);
					lua_setfield(TL, -2, "x1");
					lua_pushinteger(TL, vector.m_y1);
					lua_setfield(TL, -2, "y1");
					lua_pushinteger(TL, vector.m_index);
					lua_setfield(TL, -2, "index");
					lua_pushboolean(TL, vector.m_flags & LINE_FLAG_INTERSECTION_PRESENT);
					lua_setfield(TL, -2, "has_intersection");

					lua_settable(TL, -3);
				}
				*/
            		if (p2line.numVectors>0) {
						Vector vector = p2line.vectors[0];
						//vector.print();

						lua_createtable(TL, 0, 5);
						lua_pushinteger(TL, vector.m_x0);
						lua_setfield(TL, -2, "x0");
						lua_pushinteger(TL, vector.m_y0);
						lua_setfield(TL, -2, "y0");
						lua_pushinteger(TL, vector.m_x1);
						lua_setfield(TL, -2, "x1");
						lua_pushinteger(TL, vector.m_y1);
						lua_setfield(TL, -2, "y1");
						lua_pushinteger(TL, vector.m_index);
						lua_setfield(TL, -2, "index");
						//lua_pushboolean(TL, vector.m_flags & LINE_FLAG_INTERSECTION_PRESENT);
						//lua_setfield(TL, -2, "has_intersection");
            		} else {
                    	lua_pushnil(TL);
                    }
            }else{
            	lua_pushnil(TL);
            }

            if (cb_lines_features & LINE_INTERSECTION) {
				lua_createtable(TL, p2line.numIntersections, 0);
				for (uint8_t i = 0; i < p2line.numIntersections; i++) {
					Intersection intersection = p2line.intersections[i];
					//intersection.print();

					lua_pushinteger(TL, i+1);
					lua_createtable(TL, 0, 3);
					lua_pushinteger(TL, intersection.m_x);
					lua_setfield(TL, -2, "x");
					lua_pushinteger(TL, intersection.m_y);
					lua_setfield(TL, -2, "y");
					//lua_pushinteger(TL, intersection.m_n);
					//lua_setfield(TL, -2, "n");

					lua_createtable(TL, intersection.m_n, 0);

					for (uint8_t j = 0; j < intersection.m_n; j++) {
						IntersectionLine line = intersection.m_intLines[j];
						lua_pushinteger(TL, j+1);
						lua_createtable(TL, 0, 2);
						lua_pushinteger(TL, line.m_index);
						lua_setfield(TL, -2, "index");
						lua_pushinteger(TL, line.m_angle);
						lua_setfield(TL, -2, "angle");

						lua_settable(TL, -3);
					}
					//lua_settable(TL, -3);
					lua_setfield(TL, -2, "int_lines");

					lua_settable(TL, -3);
				}
            }else{
            	lua_pushnil(TL);
            }

            if (cb_lines_features & LINE_BARCODE) {
				lua_createtable(TL, p2line.numBarcodes, 0);
				for (uint8_t i = 0; i < p2line.numBarcodes; i++) {
					Barcode barcode = p2line.barcodes[i];
					//barcode.print();

					lua_pushinteger(TL, i+1);
					lua_createtable(TL, 0, 3);
					lua_pushinteger(TL, barcode.m_code);
					lua_setfield(TL, -2, "code");
					lua_pushinteger(TL, barcode.m_x);
					lua_setfield(TL, -2, "x");
					lua_pushinteger(TL, barcode.m_y);
					lua_setfield(TL, -2, "y");

					lua_settable(TL, -3);
				}
            }else{
            	lua_pushnil(TL);
            }

            status = lua_pcall(TL, 3, 0, 0);
            luaL_unref(TL, LUA_REGISTRYINDEX, tref);
            if (status != LUA_OK) {
		        const char *msg = lua_tostring(TL, -1);
		        lua_writestringerror("error in line features callback: %s\n", msg);
		        lua_pop(TL, 1);
                //luaL_error(TL, msg);
            }
        }
    } else {
    	printf("Error in p2line.getMainFeatures(): %i\n", ret);
    }
}


static int p2ccc_enable (lua_State *L) {
    bool enable = lua_toboolean(L, 1);
    if (enable) {
        uint32_t millis = luaL_checkinteger( L, 1 );
	    if (millis < 1) {
            lua_pushnil(L);
            lua_pushstring(L, "invalid period");
            return 2;
	    }

		cb_ccc_sigmap = luaL_optinteger(L, 2, CCC_SIG_ALL);
		cb_ccc_maxBlocks = luaL_optinteger(L, 3, 0xff);

        //set timer for callback
        p2ccc_blocks_timer = xTimerCreate("ccc_blocks", millis / portTICK_PERIOD_MS, pdTRUE,
            (void *)p2ccc_blocks_timer, callback_ccc_get_blocks);
        xTimerStart(p2ccc_blocks_timer, 0);
    } else {

         //delete timer
        xTimerStop(p2ccc_blocks_timer, portMAX_DELAY);
		xTimerDelete(p2ccc_blocks_timer, portMAX_DELAY);
    }

    lua_pushboolean(L, true);
	return 1;
}

static int p2line_enable (lua_State *L) {
    bool enable = lua_toboolean(L, 1);
    if (enable) {
        uint32_t millis = luaL_checkinteger( L, 1 );
	    if (millis < 1) {
            lua_pushnil(L);
            lua_pushstring(L, "invalid period");
            return 2;
	    }

	    if (lua_gettop(L)<4) {
            lua_pushnil(L);
            lua_pushstring(L, "missing parameters, must be (number, bool, bool, bool)");
            return 2;
	    }

	    cb_lines_features = 0;
	    if (lua_toboolean(L, 2)) cb_lines_features |= LINE_VECTOR;
	    if (lua_toboolean(L, 3)) cb_lines_features |= LINE_INTERSECTION;
	    if (lua_toboolean(L, 4)) cb_lines_features |= LINE_BARCODE;

        //set timer for callback
		p2line_feature_timer = xTimerCreate("line_features", millis / portTICK_PERIOD_MS, pdTRUE,
            (void *)p2line_feature_timer, callback_line_get_features);
        xTimerStart(p2line_feature_timer, 0);
    } else {

         //delete timer
        xTimerStop(p2line_feature_timer, portMAX_DELAY);
		xTimerDelete(p2line_feature_timer, portMAX_DELAY);
		p2line_feature_timer = NULL;
    }

    lua_pushboolean(L, true);
	return 1;
}

static int p2_get_version(lua_State *L) {
	//int8_t Pixy2CCC::getBlocks(bool wait, uint8_t sigmap, uint8_t maxBlocks)

	int8_t ret = pixy.getVersion();
	if (ret < 0) {
		lua_pushnil(L);
		lua_pushinteger(L, ret);
		return 2;
	}

	pixy.version->print();
	lua_createtable(L, 0, 5);

	lua_pushinteger(L, pixy.version->hardware);
	lua_setfield(L, -2, "hardware");
	lua_pushinteger(L, pixy.version->firmwareMajor);
	lua_setfield(L, -2, "firmwareMajor");
	lua_pushinteger(L, pixy.version->firmwareMinor);
	lua_setfield(L, -2, "firmwareMinor");
	lua_pushinteger(L, pixy.version->firmwareBuild);
	lua_setfield(L, -2, "firmwareBuild");
	lua_pushlstring(L, pixy.version->firmwareType, 10);
	lua_setfield(L, -2, "firmwareType");

	//lua_settable(L, -3);
	return 1;
}

static int p2_change_prog(lua_State *L) {
	//int8_t changeProg(const char *prog);
	const char *prog = lua_tostring(L, 1);
	int8_t ret = pixy.changeProg(prog);
	if (ret != PIXY_RESULT_OK) {
		lua_pushnil(L);
		lua_pushinteger(L, ret);
		return 2;
	}
	lua_pushboolean(L, true);
	return 1;
}

static int p2_get_resolution(lua_State *L) {
	//int8_t changeProg(const char *prog);
	int8_t ret = pixy.getResolution();
	if (ret != PIXY_RESULT_OK) {
		lua_pushnil(L);
		lua_pushinteger(L, ret);
		return 2;
	}
	lua_pushinteger(L, pixy.frameWidth);
	lua_pushinteger(L, pixy.frameHeight);
	return 2;
}

static int p2_set_camera_brightness(lua_State *L) {
	//int8_t setCameraBrightness(uint8_t brightness);
	const uint8_t brightness = lua_tonumber(L, 1);
	int8_t ret = pixy.setCameraBrightness(brightness);
	if (ret<0) {
		lua_pushnil(L);
		lua_pushinteger(L, ret);
		return 2;
	}
	lua_pushboolean(L, true);
	return 1;
}

static int p2_set_servos(lua_State *L) {
	//int8_t setServos(uint16_t s0, uint16_t s1);
	const uint16_t s0 = lua_tonumber(L, 1);
	const uint16_t s1 = lua_tonumber(L, 2);
	int8_t ret = pixy.setServos(s0, s1);
	if (ret<0) {
		lua_pushnil(L);
		lua_pushinteger(L, ret);
		return 2;
	}
	lua_pushboolean(L, true);
	return 1;
}

static int p2_set_led(lua_State *L) {
	//int8_t setLED(uint8_t r, uint8_t g, uint8_t b);
	const uint8_t r = lua_tonumber(L, 1);
	const uint8_t g = lua_tonumber(L, 2);
	const uint8_t b = lua_tonumber(L, 3);
	int8_t ret = pixy.setLED(r, g, b);
	if (ret<0) {
		lua_pushnil(L);
		lua_pushinteger(L, ret);
		return 2;
	}
	lua_pushboolean(L, true);
	return 1;
}

static int p2_set_lamp(lua_State *L) {
	//int8_t setLamp(uint8_t upper, uint8_t lower);
	const uint8_t upper = (lua_toboolean(L, 1)? 1: 0);
	const uint8_t lower = (lua_toboolean(L, 2)? 1: 0);
	int8_t ret = pixy.setLamp(upper, lower);
	if (ret<0) {
		lua_pushnil(L);
		lua_pushinteger(L, ret);
		return 2;
	}
	lua_pushboolean(L, true);
	return 1;
}

static int p2_get_fps(lua_State *L) {
	int8_t ret = pixy.getFPS();
	if (ret<0) {
		lua_pushnil(L);
		lua_pushinteger(L, ret);
		return 2;
	}
	lua_pushinteger(L, ret);
	return 1;
}

static int p2line_set_mode(lua_State *L) {
	//int8_t setLamp(uint8_t upper, uint8_t lower);
	const bool turn_delayed = lua_toboolean(L, 1);
	const bool manual_select_vector = lua_toboolean(L, 2);
	const bool white_lne = lua_toboolean(L, 3);
	uint8_t mode = 0;
	if (turn_delayed) mode |= LINE_MODE_TURN_DELAYED;
	if (manual_select_vector) mode |= LINE_MODE_MANUAL_SELECT_VECTOR;
	if (white_lne) mode |= LINE_MODE_WHITE_LINE;

	int8_t ret = p2line.setMode(mode);
	if (ret<0) {
		lua_pushnil(L);
		lua_pushinteger(L, ret);
		return 2;
	}
	lua_pushinteger(L, ret);
	return 1;
}

static int p2line_set_next_turn(lua_State *L) {
	//int8_t setNextTurn(int16_t angle);
	const int16_t angle = lua_tonumber(L, 1);

	int8_t ret = p2line.setNextTurn(angle);
	if (ret<0) {
		lua_pushnil(L);
		lua_pushinteger(L, ret);
		return 2;
	}
	lua_pushinteger(L, ret);
	return 1;
}

static int p2line_set_default_turn(lua_State *L) {
	//int8_t setDefaultTurn(int16_t angle);
	const int16_t angle = lua_tonumber(L, 1);

	int8_t ret = p2line.setDefaultTurn(angle);
	if (ret<0) {
		lua_pushnil(L);
		lua_pushinteger(L, ret);
		return 2;
	}
	lua_pushinteger(L, ret);
	return 1;
}

static int p2line_set_vector(lua_State *L) {
	//int8_t setVector(uint8_t index);
	const uint8_t index = lua_tonumber(L, 1);

	int8_t ret = p2line.setVector(index);
	if (ret<0) {
		lua_pushnil(L);
		lua_pushinteger(L, ret);
		return 2;
	}
	lua_pushinteger(L, ret);
	return 1;
}

static int p2line_reverse_vector(lua_State *L) {
	int8_t ret = p2line.reverseVector();
	if (ret<0) {
		lua_pushnil(L);
		lua_pushinteger(L, ret);
		return 2;
	}
	lua_pushinteger(L, ret);
	return 1;
}
static int p2ccc_get_blocks(lua_State *L) {
	//  int8_t getBlocks(bool wait=tr	ue, uint8_t sigmap=CCC_SIG_ALL, uint8_t maxBlocks=0xff);
	bool wait;
	if (lua_gettop(L) < 0)
		wait = true;
	else
		wait = lua_toboolean(L, 1);

	uint16_t sigmap = luaL_optinteger(L, 2, CCC_SIG_ALL);
	uint16_t maxBlocks = luaL_optinteger(L, 3, 0xff);

	int8_t ret = p2ccc.getBlocks(wait, sigmap, maxBlocks);
	if (ret < 0) {
		lua_pushboolean(L, false);
		lua_pushinteger(L, ret);
		return 2;
	}

    lua_createtable(L, p2ccc.numBlocks, 0);
	for (uint8_t i = 0; i < p2ccc.numBlocks; i++) {
		Block block = p2ccc.blocks[i];
		//block.print();

	    lua_pushinteger(L, i+1);
	    lua_createtable(L, 0, 8);

	    lua_pushinteger(L, block.m_signature);
	    lua_setfield(L, -2, "signature");
	    lua_pushinteger(L, block.m_x);
	    lua_setfield(L, -2, "x");
	    lua_pushinteger(L, block.m_y);
	    lua_setfield(L, -2, "y");
	    lua_pushinteger(L, block.m_width);
	    lua_setfield(L, -2, "width");
	    lua_pushinteger(L, block.m_height);
	    lua_setfield(L, -2, "height");
	    lua_pushinteger(L, block.m_angle);
	    lua_setfield(L, -2, "angle");
	    lua_pushinteger(L, block.m_index);
	    lua_setfield(L, -2, "index");
	    lua_pushinteger(L, block.m_age);
	    lua_setfield(L, -2, "age");

	    lua_settable(L, -3);
	}

	return 1;
}

static const luaL_Reg pixy2ccc[] = {
		{ "get_blocks", p2ccc_get_blocks },
	    { "set_blocks_callback", p2ccc_set_callback_blocks },
	    { "enable", p2ccc_enable },
		{ NULL, NULL }
};

static const luaL_Reg pixy2line[] = {
	    { "set_mode", p2line_set_mode },
	    { "set_next_turn", p2line_set_next_turn },
	    { "set_default_turn", p2line_set_default_turn },
	    { "set_vector", p2line_set_vector },
	    { "reverse_vector", p2line_reverse_vector },
	    { "set_lines_callback", p2line_set_callback_lines },
	    { "enable", p2line_enable },
		{ NULL, NULL }
};

static const luaL_Reg pixy2[] = {
		{ "init", p2_init },
		{ "get_version", p2_get_version },
		{ "change_prog", p2_change_prog },
		{ "get_resolution", p2_get_resolution },
		{ "set_camera_brightness", p2_set_camera_brightness },
		{ "set_servos", p2_set_servos },
		{ "set_led", p2_set_led },
		{ "set_lamp", p2_set_lamp },
		{ "get_fps", p2_get_fps },
		{ NULL, NULL }
};

LUALIB_API int luaopen_pixy2(lua_State *L) {
	//luaL_register(L,"vl53l0x", vl53l0x_map);
	luaL_newlib(L, pixy2);

	lua_newtable(L);
	luaL_setfuncs((L), pixy2ccc, 0);
	lua_setfield(L, -2, "ccc");

	lua_newtable(L);
	luaL_setfuncs((L), pixy2line, 0);
	lua_setfield(L, -2, "line");

	return 1;
}

MODULE_REGISTER_RAM(PIXY2, pixy2, luaopen_pixy2, 1);

#ifdef __cplusplus
}
#endif

#endif
