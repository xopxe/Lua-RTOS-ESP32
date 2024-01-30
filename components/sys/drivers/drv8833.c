/*
Program:			Motor Library

File:				Motor.cpp

Author:				John Tatum

Notes:				None
*/

/**
* @mainpage
* @section introduction Introduction
* A (very small)library for use with the Arduino environment, containing an object for the control of motors using the TI DRV8833 dual H-bridge motor driver IC. This library is contains only a single class, Motor.
*
*
* @section installation Installation
* Simply download the latest archive, and extract the contents to the "libraries" directory.
*
*
* @section license License
* The Arduino DRV8833 %Motor Controller Library is licensed under a <a href="http://opensource.org/licenses/BSD-3-Clause">Revised BSD License</a>. See the included <a href="../../license.txt">license.txt</a> for details. Note that by using this software you agree to the terms of the license.
*/

/**
* @class Motor
*
* @brief An object for the control of motors using the TI DRV8833 dual H-bridge motor driver IC
*
* @details This object encapsulates methods for the control of motors using TI DRV8833 dual H-bridge motor driver IC. This implementation was created using the Pololu carrier board available at http://www.pololu.com/catalog/product/2130. This object encapsulates methods to set the speed of a motor, as well as methods to start and stop the motor.
*/

//modified by xxopxe@gmail.com

#include "sdkconfig.h"
#if CONFIG_LUA_RTOS_LUA_USE_DRV8833

#define FRECUENCY 20

#include <drivers/drv8833.h>
#include <drivers/pwm.h>
#include <drivers/gpio.h>

static void print_driver_error(driver_error_t *error, int err) {
/*
  if (error->msg) {
  	printf(" DRIVER ERROR [%d]: type: %d, unit: %d, exc: %d msg:%s\r\n", err, error->type, error->unit, error->exception, error->msg);
  } else{
  	printf(" DRIVER ERROR [%d]: type: %d, unit: %d, exc: %d msg:NULL\r\n", err, error->type, error->unit, error->exception);
  }
*/
	printf(" DRIVER ERROR [%d]\r\n", err);
	free(error);
}

/**
* A default constructor requiring no arguments. If this constructor is used, then pins six and nine are used for the motor controller.
*
* @return a Drv8833 object using pins six and nine
*/
sDrv8833* Drv8833init_default() {
    return Drv8833init(6,9, false);
}

/**
* A constructor requiring inputs for the pins to be used for the motor controller.
*
* @param intIn1 an integer value representing the first pin used to connect to the motor controller
*
* @param intIn2 an integer value representing the second pin used to connect to the motor controller
*
* @return a Drv8833 object using the pins specified
*/
sDrv8833* Drv8833init(int intIn1, int intIn2, bool inbraked) {
    sDrv8833* drv = malloc( sizeof (sDrv8833) );
	driver_error_t *error;

	drv->pin1 = intIn1;
	drv->pin2 = intIn2;
	drv->braked = inbraked;

	if (drv->braked) {
		drv->defaultDuty = 1.0;
	}else{
		drv->defaultDuty = 0.0;
	}

	//pinMode(pin1, OUTPUT);
	//pinMode(pin2, OUTPUT);
	gpio_pin_output(drv->pin1);
	gpio_pin_output(drv->pin2);
	gpio_pin_clr(drv->pin1);
	gpio_pin_clr(drv->pin2);


	if ((error=pwm_setup(0, -1, drv->pin1, 1000 * FRECUENCY, 0, &(drv->pwm_channel1)))) {
	    	print_driver_error(error, 1);
	}
	if ((error=pwm_setup(0, -1, drv->pin2, 1000 * FRECUENCY, 0, &(drv->pwm_channel2)))) {
	    	print_driver_error(error, 2);
	}

	if ((error=pwm_set_duty(0, drv->pwm_channel1, drv->defaultDuty))) {
	    	print_driver_error(error, 3);
	}
	if ((error=pwm_set_duty(0, drv->pwm_channel2, drv->defaultDuty))) {
	    	print_driver_error(error, 4);
	}

	drv->intSpeed = 0;
	drv->isRunning = false;
	
	return drv;
}

/*I probably need to add a destructor here to call stopMotor(), but I no longer have this motor controller available so I will wait until I have another one so that I can test this change.*/

/**
* A method to set the speed of the motor. This method does not start or stop the motor, however, if the motor is already running it will "re-start" the motor at the speed specified.
*
* @param intIn an integer value between -100 and 100(inclusive) representing the speed at which the motor will run. This value roughly equates to the PWM duty cycle(ie, zero is off and 100 is full speed), with negative and positive values having opposite rotations.
*
* @return NA
*/
void Drv8833setMotorSpeed(sDrv8833 *drv, int intIn) {
	driver_error_t *error;
	if ((intIn >= -100) && (intIn <= 100)) {
		drv->intSpeed = intIn;
	}

	double duty1 = drv->defaultDuty;
	double duty2 = drv->defaultDuty;
	double speed = drv->intSpeed/100.0;

	/*The following is to change the speed if the motor is already running*/
	if (drv->isRunning) {

		if (drv->braked){
			if (drv->intSpeed < 0) {
				duty2 = 1.0 + speed;
			} else if (drv->intSpeed > 0) {
				duty1 = 1.0 - speed ;
			}

		}else{

			if (drv->intSpeed < 0) {
				duty1 = -speed;
			} else if (drv->intSpeed > 0) {
				duty2 = speed ;
			}

		}

		if ((error=pwm_set_duty(0, drv->pwm_channel1, duty1))) {
					print_driver_error(error, 15);
		}
		if ((error=pwm_set_duty(0, drv->pwm_channel2, duty2))) {
					print_driver_error(error, 16);
		}

	}
}

/**
* This method returns the value of the current speed for a Drv8833 object. This value cannot be used to authoritatively determine whether or not the motor is running. To authoritatively determine whether or not the motor is running, call isMotorRunning().
*
* @return an integer between -100 and 100(inclusive) representing the value of the current speed for the Drv8833 object.
*/
int Drv8833getMotorSpeed(sDrv8833 *drv) {
	return drv->intSpeed;
}

/**
* This method returns the current run state of the motor(true is running, false is not running).
*
* @return a boolean value representing whether or not the motor is running.
*/
bool Drv8833isMotorRunning(sDrv8833 *drv) {
	return drv->isRunning;
}

/**
* This method stops the motor. Specifically, it sets both output pins low.
*
* @return NA
*/
void Drv8833stopMotor(sDrv8833 *drv) {
	driver_error_t *error;
	//digitalWrite(pin1, LOW);
	//digitalWrite(pin2, LOW);
	if ((error=pwm_stop(0, drv->pwm_channel1))) {
	    	print_driver_error(error, 31);
	}
	if ((error=pwm_stop(0, drv->pwm_channel2))) {
	    	print_driver_error(error, 32);
	}

	gpio_pin_clr(drv->pin1);
	gpio_pin_clr(drv->pin2);

	drv->isRunning = false;
}

/**
* This method starts the motor, which will run in the direction and speed determined by the object's speed property. To get the current motor speed call getMotorSpeed(). To set the motor's speed, call setMotorSpeed(int).
*
* @return NA
*/
void Drv8833startMotor(sDrv8833 *drv) {
	driver_error_t *error;

	if ((error=pwm_start(0, drv->pwm_channel1))) {
	    	print_driver_error(error, 41);
	}
	if ((error=pwm_start(0, drv->pwm_channel2))) {
	    	print_driver_error(error, 42);
	}

	if ((error=pwm_set_duty(0, drv->pwm_channel1, drv->defaultDuty))) {
				print_driver_error(error, 43);
	}
	if ((error=pwm_set_duty(0, drv->pwm_channel2, drv->defaultDuty))) {
				print_driver_error(error, 44);
	}

	drv->isRunning = true;
}

#endif
