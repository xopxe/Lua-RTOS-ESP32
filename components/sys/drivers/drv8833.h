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

class Drv8833 {
	private:
	int intSpeed;
	int pin1;
	int pin2;
	bool isRunning;
	bool braked;
	double defaultDuty;

    int8_t pwm_channel1;
    int8_t pwm_channel2;

	public:
	Drv8833();
	Drv8833(int intIn1, int intIn2, bool braked);
	void setMotorSpeed(int intIn);
	int getMotorSpeed();
	bool isMotorRunning();
	void stopMotor();
	void startMotor();
};
#endif
