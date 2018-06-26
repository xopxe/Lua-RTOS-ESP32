#include "sdkconfig.h"
#if CONFIG_LUA_RTOS_LUA_USE_OMNIHBRIDGE

#define NMOTORS 3
#define MOTOR_PINS {25,26, 27,28, 29,30}

#define OMNI_CTRL_TIMER 50 //ms

/*
#define SERVO_CW_VEL_MIN 1
#define SERVO_CW_DTY_MIN 1400
#define SERVO_CW_VEL_MAX 90
#define SERVO_CW_DTY_MAX 500
#define SERVO_CCW_VEL_MIN -1
#define SERVO_CCW_DTY_MIN 1600
#define SERVO_CCW_VEL_MAX -90
#define SERVO_CCW_DTY_MAX 2500
*/

#ifdef __cplusplus
extern "C"{
#endif

#include "freertos/FreeRTOS.h"
#include "freertos/adds.h"

#include "modules.h"

#include <drivers/cpu.h>
#include "freertos/timers.h"

#ifdef __cplusplus
  #include "lua.hpp"
#else
  #include "lua.h"
  #include "lualib.h"
  #include "lauxlib.h"
#endif

#include <drivers/drv8833.h>
#include "vector_math.h"

typedef struct {
	Drv8833 *driver;
    float target_v;
    float current_v;
} servo_t;

TimerHandle_t motor_control_timer;

static servo_t motors[NMOTORS];
float robot_r;

SF3dVector static getW(float x_dot, float y_dot, float w_dot, float phi_r){
	SF3dVector v(x_dot, y_dot, w_dot);

	SF3dMatrix M(
		-sin(phi_r),         cos(phi_r),        robot_r,
		-sin(PI/3 - phi_r), -cos(PI/3 - phi_r), robot_r,
		 sin(PI/3 + phi_r), -cos(PI/3 + phi_r), robot_r
	);

	SF3dVector w = M*v;
	return w;
}

/*
float static lin_interpolation(float x0, float y0, float x1, float y1, float x) {
    return y0+(x-x0)*(y1-y0)/(x1-x0);
}
int static pulse_from_angle(float a) {
    if (a>1.0) {
        return roundl(lin_interpolation(1.0, 1400, 90.0, 500, a));
    }
    if (a<-1.0) {
        return roundl(lin_interpolation(-1.0, 1600, -90.0, 2500, a));
    }
    return 1500;
}
*/

static void callback_sw_func(TimerHandle_t xTimer) {
//FIXME implementar PID
    for (int i=0; i<NMOTORS; i++) {
        bool dirty = false;
        if (motors[i].current_v < motors[i].target_v) {
            dirty=true;
            motors[i].current_v+=10;
            if (motors[i].current_v > motors[i].target_v) 
                motors[i].current_v = motors[i].target_v;
        }
        if (motors[i].current_v > motors[i].target_v) {
            dirty=true;
            motors[i].current_v-=10;
            if (motors[i].current_v < motors[i].target_v) 
                motors[i].current_v = motors[i].target_v;
        }
        if (dirty) {
            motors[i].driver->setMotorSpeed(motors[i].current_v);
        }
    }

}


static int omni_init (lua_State *L) {
    int8_t default_pins[] = MOTOR_PINS;

    robot_r = luaL_checknumber(L, 1);

    for (int i=0; i<NMOTORS; i++) {
        int8_t pin1 = luaL_optinteger( L, (2*i)+2, default_pins[2*i] );
        int8_t pin2 = luaL_optinteger( L, (2*i)+3, default_pins[2*i+1] );

        printf("omni Setting motor %d on pins %d,%d", i, pin1, pin2);
        motors[i].driver=new Drv8833(pin1, pin2);
        printf(" done\r\n");
        motors[i].current_v=0;
        motors[i].target_v=0;
    }

    motor_control_timer = xTimerCreate("omni_hbridge", OMNI_CTRL_TIMER / portTICK_PERIOD_MS, pdTRUE, 
                            (void *)motor_control_timer, callback_sw_func);
    /*xTimerStart(motor_control_timer, 0);*/

    lua_pushboolean(L, true);
	return 1;
}

static int omni_set_enable (lua_State *L) {
    bool success = true;
    bool enable = lua_gettop(L)==0 || lua_toboolean( L, 1 );

    if (enable) {
        xTimerStart(motor_control_timer, 0);
        for (int i=0; i<NMOTORS; i++) {
            motors[i].current_v=0;
            motors[i].driver->startMotor();
        }
    } else {
        xTimerStop(motor_control_timer, 0);
        for (int i=0; i<NMOTORS; i++) {
            motors[i].current_v=0;
            motors[i].driver->stopMotor();
        }
    }

    lua_pushboolean(L, success);
	return 1;
}


static int omni_raw_write (lua_State *L) {
    for (int i=0; i<NMOTORS; i++) {
        double value = luaL_optnumber( L, i+1, 0 );
        motors[i].driver->setMotorSpeed(value);
    }

    lua_pushboolean(L, true);
	return 1;
}

static int omni_drive (lua_State *L) {
    float x_dot = luaL_checknumber( L, 1 );
    float y_dot = luaL_checknumber( L, 2 );
    float w_dot = luaL_checknumber( L, 3 );
    float phi = luaL_optnumber( L, 4, 0.0 );

	SF3dVector w = getW(x_dot, y_dot, w_dot, phi);
    //printf("omni computed vel %f %f %f\r\n", w.x, w.y, w.z);

    //printf("omni setting duty %d %d %d\r\n", px, py, pw);

    motors[0].target_v = w.x;
    motors[1].target_v = w.y;
    motors[2].target_v = w.z;

    lua_pushboolean(L, true);
	return 1;
}


static const luaL_Reg omni_hbridge[] = {
//	{"attach", lvl53l0x_attach},
//	{"detach", lvl53l0x_detach},
	{"init", omni_init},
	{"raw_write", omni_raw_write},
	{"drive", omni_drive},
	{"set_enable", omni_set_enable},
    {NULL, NULL}
};

LUALIB_API int luaopen_omni_hbridge( lua_State *L ) {
    //luaL_register(L,"vl53l0x", vl53l0x_map);
    luaL_newlib(L, omni_hbridge);
	return 1;
}

MODULE_REGISTER_RAM(OMNIHBRIDGE, omni_hbridge, luaopen_omni_hbridge, 1);


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
