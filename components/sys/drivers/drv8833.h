/*
Program:			Motor Library

File:				Motor.h

Author:				John Tatum

Notes:				None
*/
#ifndef Drv8833_h
#define Drv8833_h

//#include <Arduino.h>
//#include <WProgram.h>

#include <stdbool.h>
#include <stdint.h>

typedef struct sDrv8833 {
	int intSpeed;
	int pin1;
	int pin2;
	bool isRunning;
	bool braked;
	double defaultDuty;

    int8_t pwm_channel1;
    int8_t pwm_channel2;
} sDrv8833;

sDrv8833 *Drv8833init_default();
sDrv8833 *Drv8833init(int intIn1, int intIn2, bool braked);
void Drv8833setMotorSpeed(sDrv8833 *drv, int intIn);
int  Drv8833getMotorSpeed(sDrv8833 *drv);
bool Drv8833isMotorRunning(sDrv8833 *drv);
void Drv8833stopMotor(sDrv8833 *drv);
void Drv8833startMotor(sDrv8833 *drv);

#endif
