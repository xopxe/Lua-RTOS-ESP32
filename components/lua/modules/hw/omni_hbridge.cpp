#include "sdkconfig.h"
#if CONFIG_LUA_RTOS_LUA_USE_OMNIHBRIDGE

#define NMOTORS 3
#define MOTOR_PINS {25,26, 27,28, 29,30}
#define MOTOR_ENC  {39,37, 38,36, 34,35}

#define MOTORS_BRAKED true

#define OMNI_NRO_TIMER CPU_TIMER0
#define OMNI_CTRL_TIMER 0.05 // s

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
#include <drivers/timer.h>

#ifdef __cplusplus
  #include "lua.hpp"
#else
  #include "lua.h"
  #include "lualib.h"
  #include "lauxlib.h"
#endif

#include <drivers/drv8833.h>
#include "vector_math.h"

#include "error.h"
#include <drivers/encoder.h>

typedef struct {
	Drv8833 *driver;

    encoder_h_t *encoder;
    int32_t counter;

    float target_v;
    float accum_error;
    float prev_error;
    float output;

} servo_t;

typedef struct {
    float x;
    float y;
    float phi;

} odom_t;

float Kp = 0.0;
float Ki = 0.0;
float Kd = 0.0;
float KF = 1.0;

float Rad_per_tick = 0.0;
float Wheel_diameter = 0.0;
float Wheel_radius = 0.0;
float m_per_sec_to_tics_per_sec = 1.0;
int cont_c = 1;

float Max_output = 100.0;

int odom_period_factor = 1.0;

TimerHandle_t motor_control_timer;

int encoder_lua_callback = LUA_NOREF;
int direct_kinematic_lua_callback = LUA_NOREF;

static servo_t motors[NMOTORS];

static odom_t odometry;
float tics_motores[NMOTORS];

float robot_r;

SF3dVector static getW(float x_dot, float y_dot, float w_dot, float phi_r){
	SF3dVector v(x_dot, y_dot, w_dot);
  float pi_3 = PI/3;
	SF3dMatrix M(
		-sin(phi_r),         cos(phi_r),        robot_r,
		-sin(pi_3 - phi_r), -cos(pi_3 - phi_r), robot_r,
		 sin(pi_3 + phi_r), -cos(pi_3 + phi_r), robot_r
	);

	SF3dVector w = M*v;
	return w;
}

SF3dVector static getInverseW(float w_1, float w_2, float w_3, float phi){

	SF3dVector x(w_1 *Wheel_radius, w_2 *Wheel_radius, w_3 *Wheel_radius);

  float pi_6 = PI/6;
  float dostercios = 2.0/3;
  float robot_r_3 = 1.0/(robot_r*3);

	SF3dMatrix A(
    -sin(phi)*dostercios , -cos(pi_6 + phi)*dostercios, cos(phi - pi_6)*dostercios,
		cos(phi)*dostercios  , -sin(pi_6 + phi)*dostercios, sin(phi - pi_6)*dostercios,
		robot_r_3            , robot_r_3                  , robot_r_3
	);

	SF3dVector u = A*x;
	return u;
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

static void motor_control_callback(TimerHandle_t xTimer) {
//FIXME implementar PID


    for (int i=0; i<NMOTORS; i++) {
    //for (int i=2; i<NMOTORS; i++) {
        servo_t *m = &(motors[i]);

        //compute v (m/s)
        float current_v  = m->counter / OMNI_CTRL_TIMER;  // tics/s
        tics_motores[i] += m->counter;
        //printf("motor %i, tics: %i, current_v: %f\n", i, m->counter, current_v);
        m->counter = 0;


        float error = m->target_v - current_v;
        float accum_error = m->accum_error+error;

        /*
        float Iterm = Ki * accum_error;
        if (Iterm > Max_output) {
            Iterm = Max_output;
        } else if (Iterm < -Max_output) {
            Iterm = -Max_output;
        }
        */

        m->output = KF * m->target_v;
        m->output += Kp * error;
        m->output += Ki * accum_error;
        m->output += Kd * (error - m->prev_error);

        if (m->output>Max_output) {
            m->output=Max_output;
        } else if (m->output<-Max_output) {
            m->output=-Max_output;
        } else {
            m->accum_error = accum_error; // only here, for windup protection
        }


        m->prev_error = error;
        // if (i == 1){
        //   printf("motor %i, target_v %f, current_v %f, output %f, error %f \n",i, m->target_v, current_v, m->output, error);
        // }

        // m->driver->setMotorSpeed(m->output);
    }

    for (int i=0; i<NMOTORS; i++) {
        motors[i].driver->setMotorSpeed(motors[i].output);
    }

    cont_c++;

    lua_State *TL;
    lua_State *L;
    int tref;

    if ((direct_kinematic_lua_callback != LUA_NOREF) && (cont_c % odom_period_factor == 0)) {
      odom_t *o = &(odometry);
      float tics_to_rad_s = Rad_per_tick/(OMNI_CTRL_TIMER*odom_period_factor);
      SF3dVector odom_vels = getInverseW(tics_motores[0]*tics_to_rad_s, tics_motores[1]*tics_to_rad_s, tics_motores[2]*tics_to_rad_s, o->phi);

      o->x += odom_vels.x;
      o->y += odom_vels.y;
      o->phi += odom_vels.z;

      cont_c = 0; // reseteo el contador para respetar la cantidad de controles.

      // reseteo contadores
      tics_motores[0] = 0;
      tics_motores[1] = 0;
      tics_motores[2] = 0;

      //Devuelvo la odometria
      L = pvGetLuaState();
      TL = lua_newthread(L);

      tref = luaL_ref(L, LUA_REGISTRYINDEX);

      lua_rawgeti(L, LUA_REGISTRYINDEX, direct_kinematic_lua_callback);
      lua_xmove(L, TL, 1);
      lua_pushnumber(TL, o->x);
      lua_pushnumber(TL, o->y);
      lua_pushnumber(TL, o->phi);   //*Rad_per_tick
      lua_pushnumber(TL, odom_vels.x);
      lua_pushnumber(TL, odom_vels.y);
      lua_pushnumber(TL, odom_vels.z);
      int status = lua_pcall(TL, 6, 0, 0);
      luaL_unref(TL, LUA_REGISTRYINDEX, tref);

      if (status != LUA_OK) {
        const char *msg = lua_tostring(TL, -1);
          //luaL_error(TL, msg);
        lua_writestringerror("error in odometry callback %s\n", msg);
        lua_pop(TL, 1);
      }

    }


/*
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
*/
}

static void callback_enc_func(int i_encoder, int8_t dir, uint32_t counter, uint8_t button) {
    //printf("motor %i, dir %i, counter %i\n", i_encoder, dir, counter);
    motors[i_encoder].counter+=dir;

    lua_State *TL;
  	lua_State *L;
  	int tref;

  	if (encoder_lua_callback != LUA_NOREF) {

  	    L = pvGetLuaState();
  	    TL = lua_newthread(L);

  	    tref = luaL_ref(L, LUA_REGISTRYINDEX);

  	    lua_rawgeti(L, LUA_REGISTRYINDEX, encoder_lua_callback);
  	    lua_xmove(L, TL, 1);
          lua_pushinteger(TL, i_encoder+1);
          lua_pushinteger(TL, dir);
          lua_pushinteger(TL, counter);   //*Rad_per_tick
  	    int status = lua_pcall(TL, 3, 0, 0);
          luaL_unref(TL, LUA_REGISTRYINDEX, tref);

          if (status != LUA_OK) {
  	    	const char *msg = lua_tostring(TL, -1);
          	//luaL_error(TL, msg);
      		lua_writestringerror("error in encoder callback %s\n", msg);
  		    lua_pop(TL, 1);
          }
  	}

}



static int omni_init (lua_State *L) {
	driver_error_t *error;
    int8_t default_pins[] = MOTOR_PINS;
    int8_t default_enc[] = MOTOR_ENC;

    robot_r = luaL_checknumber(L, 1);

    for (int i=0; i<NMOTORS; i++) {
        int8_t pin1 = luaL_optinteger( L, (4*i)+2, default_pins[2*i] );
        int8_t pin2 = luaL_optinteger( L, (4*i)+3, default_pins[2*i+1] );
        int8_t encA = luaL_optinteger( L, (4*i)+4, default_enc[2*i] );
        int8_t encB = luaL_optinteger( L, (4*i)+5, default_enc[2*i+1] );

        printf("omni Setting motor %d pins:%d,%d enc:%d,%d", i, pin1, pin2, encA, encB);

        //driver
        motors[i].driver=new Drv8833(pin1, pin2, MOTORS_BRAKED);

        //encoder
        encoder_h_t *encoder;
        if ((error = encoder_setup(encA, encB, -1, &encoder))) {
        	return luaL_driver_error(L, error);
        }
        if ((error = encoder_register_callback(encoder, callback_enc_func, i, 1))) {
        	return luaL_driver_error(L, error);
        }
        motors[i].encoder=encoder;

        printf(" done\r\n");
        motors[i].target_v=0;
        tics_motores[i] = 0;
    }
    odometry.x = 0;
    odometry.y = 0;
    odometry.phi = 0;
    // motor_control_timer = xTimerCreate("omni_hbridge", 1000*OMNI_CTRL_TIMER / portTICK_PERIOD_MS, pdTRUE,
    //                         (void *)motor_control_timer, callback_sw_func);
    /*xTimerStart(motor_control_timer, 0);*/
    if ((error = tmr_setup(OMNI_NRO_TIMER, 1000*1000*OMNI_CTRL_TIMER, motor_control_callback, 1))) {
        return luaL_driver_error(L, error);
    }
    lua_pushboolean(L, true);
	return 1;
}

static int omni_set_enable (lua_State *L) {
    driver_error_t *error;
    bool success = true;
    bool enable = lua_gettop(L)==0 || lua_toboolean( L, 1 );

    if (enable) {
        // xTimerStart(motor_control_timer, 0);
        if ((error = tmr_start(OMNI_NRO_TIMER))) {
            return luaL_driver_error(L, error);
        }
        for (int i=0; i<NMOTORS; i++) {
            motors[i].driver->startMotor();
            motors[i].counter = 0;
        }
    } else {
        // xTimerStop(motor_control_timer, 0);
        if ((error = tmr_stop(OMNI_NRO_TIMER))) {
            return luaL_driver_error(L, error);
        }
        for (int i=0; i<NMOTORS; i++) {
            motors[i].driver->stopMotor();
        }
    }

    lua_pushboolean(L, success);
	return 1;
}

static int omni_set_raw (lua_State *L) {
    bool success = true;
    bool enable = lua_gettop(L)==0 || lua_toboolean( L, 1 );

    if (enable) {
        xTimerStop(motor_control_timer, 0);
        for (int i=0; i<NMOTORS; i++) {
            motors[i].driver->startMotor();
        }
    } else {
        for (int i=0; i<NMOTORS; i++) {
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

static int omni_set_pid (lua_State *L) {
    Kp = luaL_optnumber( L, 1, 1.0 );
    Ki = luaL_optnumber( L, 2, 0.0 );
    Kd = luaL_optnumber( L, 3, 0.0 );
    KF = luaL_optnumber( L, 4, 1.0 );
    lua_pushboolean(L, true);
	return 1;
}

static int omni_set_rad_per_tick (lua_State *L) {
    Rad_per_tick = luaL_optnumber( L, 1, 1.0 );
    m_per_sec_to_tics_per_sec = 1/(Rad_per_tick * Wheel_radius);
    lua_pushboolean(L, true);
	return 1;
}

static int omni_set_wheel_diameter (lua_State *L) {
    Wheel_diameter = luaL_optnumber( L, 2, 0.038);
    Wheel_radius = Wheel_diameter /2;
    m_per_sec_to_tics_per_sec = 1/(Rad_per_tick * Wheel_radius);
    lua_pushboolean(L, true);
	return 1;
}

static int omni_set_max_output (lua_State *L) {
    Max_output = luaL_optnumber( L, 1, 100.0 );
    lua_pushboolean(L, true);
	return 1;
}

static int omni_drive (lua_State *L) {
    float x_dot = luaL_checknumber( L, 1 );
    float y_dot = luaL_checknumber( L, 2 );
    float w_dot = luaL_checknumber( L, 3 );
    float phi = luaL_optnumber( L, 4, 0.0 );

    SF3dVector w = getW(x_dot, y_dot, w_dot, phi);
    //printf("omni computed vel 1 %f %f \r\n", w.x, w.x * m_per_sec_to_tics_per_sec);

    motors[0].target_v = w.x * m_per_sec_to_tics_per_sec;
    motors[1].target_v = w.y * m_per_sec_to_tics_per_sec;
    motors[2].target_v = w.z * m_per_sec_to_tics_per_sec;

    motors[0].accum_error = 0;
    motors[1].accum_error = 0;
    motors[2].accum_error = 0;

    lua_pushboolean(L, true);
	return 1;
}

static int omni_set_encoder_callback( lua_State* L ) {
	if (lua_isfunction(L, 1)) {
		luaL_checktype(L, 1, LUA_TFUNCTION);
		lua_pushvalue(L, 1);
		encoder_lua_callback = luaL_ref(L, LUA_REGISTRYINDEX);
	} else {
		encoder_lua_callback = LUA_NOREF;
	}

    return 1;
}

static int omni_set_odometry_callback( lua_State* L ) {
	if (lua_isfunction(L, 1)) {
		luaL_checktype(L, 1, LUA_TFUNCTION);
    odom_period_factor = luaL_optnumber(L, 2, 10); // Number of periods to publish odom.
    odometry.x = luaL_optnumber(L, 3, 0.0);
    odometry.y = luaL_optnumber(L, 4, 0.0);
    odometry.phi = luaL_optnumber(L, 5, 0.0);
		lua_pushvalue(L, 1);
		direct_kinematic_lua_callback = luaL_ref(L, LUA_REGISTRYINDEX);
	} else {
		direct_kinematic_lua_callback = LUA_NOREF;
	}

    return 1;
}


static const luaL_Reg omni_hbridge[] = {
//	{"attach", lvl53l0x_attach},
//	{"detach", lvl53l0x_detach},
	{"init", omni_init},
	{"raw_write", omni_raw_write},
	{"drive", omni_drive},
	{"set_enable", omni_set_enable},
	{"set_raw", omni_set_raw},
	{"set_pid", omni_set_pid},
  {"set_max_output", omni_set_max_output},
  {"set_set_rad_per_tick", omni_set_rad_per_tick},
  {"set_set_wheel_diameter", omni_set_wheel_diameter},
  {"set_encoder_callback", omni_set_encoder_callback},
  {"set_odometry_callback",omni_set_odometry_callback},

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
