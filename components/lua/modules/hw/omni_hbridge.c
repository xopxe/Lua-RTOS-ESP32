#include "sdkconfig.h"
#if CONFIG_LUA_RTOS_LUA_USE_OMNIHBRIDGE

#define NMOTORS 3
#define MOTOR_PINS {25,26, 27,28, 29,30}
#define MOTOR_ENC  {39,37, 38,36, 34,35}

#define MOTORS_BRAKED true

#define OMNI_NRO_TIMER CPU_TIMER0
#define OMNI_CTRL_TIMER 0.05 // s


#define M_PI_6 M_PI/6.0
#define M_PI_3 M_PI/3.0
#define M_2_3 2.0/3.0

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
	sDrv8833 *driver;

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
float m_per_sec_to_tics_per_sec = 0.0;
unsigned int cont_c = 1;

float Max_output = 100.0;

int odom_period_factor = 5;

TimerHandle_t motor_control_timer;

int encoder_lua_callback = LUA_NOREF;
int direct_kinematic_lua_callback = LUA_NOREF;

bool distance_limit_set = false;
static float distance_limit_sq = 0.0;
bool rotation_limit_set = false;
static float rotation_limit = 0.0;
int limits_lua_callback = LUA_NOREF;
bool stop_on_limit = false;

static servo_t motors[NMOTORS];

static odom_t odometry;
static odom_t limit_reference;
float tics_motores[NMOTORS];

float robot_r;
float robot_r_3;

vec3_t static getW(float x_dot, float y_dot, float w_dot, float phi_r){
	vec3_t v = vec3(x_dot, y_dot, w_dot);

	mat3_t M = mat3(
		-sin(phi_r),         cos(phi_r),        robot_r,
		-sin(M_PI_3 - phi_r), -cos(M_PI_3 - phi_r), robot_r,
		 sin(M_PI_3 + phi_r), -cos(M_PI_3 + phi_r), robot_r
	);

	vec3_t w = mat3_mul_vec3(M,v);
	return w;
}

vec3_t static getInverseW(float w_1, float w_2, float w_3, float phi){
	vec3_t x = vec3(w_1 *Wheel_radius, w_2 *Wheel_radius, w_3 *Wheel_radius);


	mat3_t A = mat3(
        -sin(phi)*M_2_3 , -cos(M_PI_6 + phi)*M_2_3, cos(phi - M_PI_6)*M_2_3,
		cos(phi)*M_2_3  , -sin(M_PI_6 + phi)*M_2_3, sin(phi - M_PI_6)*M_2_3,
        robot_r_3       , robot_r_3               , robot_r_3
	);

	vec3_t u = mat3_mul_vec3(A,x);
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
        Drv8833setMotorSpeed(motors[i].driver, motors[i].output);
    }

    cont_c++;
  
  
    //printf ("[%d %d]",cont_c, odom_period_factor);
    if ( (cont_c % odom_period_factor == 0) &&
         ((direct_kinematic_lua_callback != LUA_NOREF) || distance_limit_set || rotation_limit_set) ) {

        lua_State *TL;
        lua_State *L;
        int tref;

        cont_c = 0; // reseteo el contador para respetar la cantidad de controles.
        float dt = OMNI_CTRL_TIMER*odom_period_factor;
         
        float tics_to_rad_s = Rad_per_tick/dt;
        vec3_t odom_vels = getInverseW(
            tics_motores[0]*tics_to_rad_s, 
            tics_motores[1]*tics_to_rad_s, 
            tics_motores[2]*tics_to_rad_s, 
            odometry.phi
        );

        odometry.x += (odom_vels.x * dt);
        odometry.y += (odom_vels.y * dt);
        odometry.phi += (odom_vels.z * dt);

        // reseteo contadores
        tics_motores[0] = 0;
        tics_motores[1] = 0;
        tics_motores[2] = 0;
        
        bool trigger_distance = false;
        if (distance_limit_set) {
            float dx = odometry.x - limit_reference.x;
            float dy = odometry.y - limit_reference.y;
            float distance_sq = dx*dx + dy*dy;
            if (distance_sq>=distance_limit_sq) {
                trigger_distance = true;
            }
        }
        if (rotation_limit_set) {
        float angle = fabs(odometry.phi - limit_reference.phi);
            if ( angle >= rotation_limit) {
                trigger_distance = true;
            }
        }        
        if ( trigger_distance ) {
            if ( stop_on_limit ) {
                motors[0].target_v = 0;
                motors[1].target_v = 0;
                motors[2].target_v = 0;

                //FIXME neccesary?
                motors[0].accum_error = 0;
                motors[1].accum_error = 0;
                motors[2].accum_error = 0;
            }
              
            limit_reference = odometry;
            
            if ( limits_lua_callback != LUA_NOREF ) { 

                L = pvGetLuaState();
                TL = lua_newthread(L);

                tref = luaL_ref(L, LUA_REGISTRYINDEX);

                lua_rawgeti(L, LUA_REGISTRYINDEX, limits_lua_callback);
                lua_xmove(L, TL, 1);

                lua_pushnumber(TL, odometry.x);
                lua_pushnumber(TL, odometry.y);
                lua_pushnumber(TL, odometry.phi);   //*Rad_per_tick
                int status = lua_pcall(TL, 3, 0, 0);
                luaL_unref(TL, LUA_REGISTRYINDEX, tref);   
                if (status != LUA_OK) {
                    const char *msg = lua_tostring(TL, -1);
                    lua_writestringerror("error in limits callback %s\n", msg);
                    lua_pop(TL, 1);
                }
            }
        }

        if ( direct_kinematic_lua_callback != LUA_NOREF ) {

            //Devuelvo la odometria
            L = pvGetLuaState();
            TL = lua_newthread(L);

            tref = luaL_ref(L, LUA_REGISTRYINDEX);

            lua_rawgeti(L, LUA_REGISTRYINDEX, direct_kinematic_lua_callback);
            lua_xmove(L, TL, 1);
            lua_pushnumber(TL, odometry.x);
            lua_pushnumber(TL, odometry.y);
            lua_pushnumber(TL, odometry.phi);   //*Rad_per_tick
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
    }

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
    robot_r_3 = 1.0/(robot_r*3.0);

    for (int i=0; i<NMOTORS; i++) {
        int8_t pin1 = luaL_optinteger( L, (4*i)+2, default_pins[2*i] );
        int8_t pin2 = luaL_optinteger( L, (4*i)+3, default_pins[2*i+1] );
        int8_t encA = luaL_optinteger( L, (4*i)+4, default_enc[2*i] );
        int8_t encB = luaL_optinteger( L, (4*i)+5, default_enc[2*i+1] );

        printf("omni Setting motor %d pins:%d,%d enc:%d,%d", i, pin1, pin2, encA, encB);

        //driver
        motors[i].driver = Drv8833init(pin1, pin2, MOTORS_BRAKED);

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
            Drv8833startMotor(motors[i].driver);
            motors[i].counter = 0;
        }
    } else {
        // xTimerStop(motor_control_timer, 0);
        if ((error = tmr_stop(OMNI_NRO_TIMER))) {
            return luaL_driver_error(L, error);
        }
        for (int i=0; i<NMOTORS; i++) {
            Drv8833stopMotor(motors[i].driver);
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
            Drv8833startMotor(motors[i].driver);
        }
    } else {
        for (int i=0; i<NMOTORS; i++) {
            Drv8833stopMotor(motors[i].driver);
        }
    }

    lua_pushboolean(L, success);
	return 1;
}



static int omni_raw_write (lua_State *L) {
    for (int i=0; i<NMOTORS; i++) {
        double value = luaL_optnumber( L, i+1, 0 );
        Drv8833setMotorSpeed(motors[i].driver,value);
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
    Rad_per_tick = luaL_checknumber( L, 1 );
    //m_per_sec_to_tics_per_sec = 1/(Rad_per_tick * Wheel_radius);
    lua_pushboolean(L, true);
	return 1;
}

static int omni_set_wheel_diameter (lua_State *L) {
    Wheel_diameter = luaL_checknumber(L, 1); //luaL_optnumber( L, 1, 0.038);
    Wheel_radius = Wheel_diameter/2.0;
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

    vec3_t w = getW(x_dot, y_dot, w_dot, phi);
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
        odom_period_factor = luaL_optnumber(L, 2, 5); // Number of periods to publish odom.
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

static int omni_set_limits( lua_State* L ) {
    if (lua_isnumber(L,1))  {
        distance_limit_sq = lua_tonumber( L, 1 );
        distance_limit_sq *= distance_limit_sq;
        limit_reference = odometry;  
        distance_limit_set = true;
    } else {
        distance_limit_set = false;
    }
    if (lua_isnumber(L,2))  {
        rotation_limit = lua_tonumber( L, 2 );
        limit_reference = odometry;
        rotation_limit_set = true;
    } else {
        rotation_limit_set = false;
    }
    stop_on_limit = lua_toboolean(L,3);
    if (lua_isfunction(L, 4)) {
        lua_pushvalue(L, 4);
        limits_lua_callback = luaL_ref(L, LUA_REGISTRYINDEX);
    } else {
	    limits_lua_callback = LUA_NOREF;
    }
    lua_pushboolean(L, true);
    return 1;
}

static const LUA_REG_TYPE omni_hbridge_map[] = {
	{LSTRKEY("init"), LFUNCVAL(omni_init)},
	{LSTRKEY("raw_write"), LFUNCVAL(omni_raw_write)},
	{LSTRKEY("drive"), LFUNCVAL(omni_drive)},
	{LSTRKEY("set_enable"), LFUNCVAL(omni_set_enable)},
	{LSTRKEY("set_raw"), LFUNCVAL(omni_set_raw)},
	{LSTRKEY("set_pid"), LFUNCVAL(omni_set_pid)},
    {LSTRKEY("set_max_output"), LFUNCVAL(omni_set_max_output)},
    {LSTRKEY("set_set_rad_per_tick"), LFUNCVAL(omni_set_rad_per_tick)},
    {LSTRKEY("set_set_wheel_diameter"), LFUNCVAL(omni_set_wheel_diameter)},
    {LSTRKEY("set_encoder_callback"), LFUNCVAL(omni_set_encoder_callback)},
    {LSTRKEY("set_odometry_callback"),LFUNCVAL(omni_set_odometry_callback)},
    {LSTRKEY("set_limits"), LFUNCVAL(omni_set_limits)},

    { LNILKEY, LNILVAL }
};

LUALIB_API int luaopen_omni_hbridge( lua_State *L ) {
    //luaL_register(L,"vl53l0x", vl53l0x_map);
	LNEWLIB(L, omni_hbridge_map);   
}

MODULE_REGISTER_ROM(OMNIHBRIDGE, omni_hbridge, omni_hbridge_map, luaopen_omni_hbridge, 1);


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

#endif
