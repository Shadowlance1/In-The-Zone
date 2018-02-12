typedef struct Base {
	tMotor frontLeftBaseMotor;
	tMotor frontRightBaseMotor;
	tMotor backLeftBaseMotor;
	tMotor backRightBaseMotor;

	tSensors myGyro;

	float leftCompensation;
	float rightCompensation;
	float radius;
	float encoderTickCount;

	bool isHolonomicBase;
	bool isFourMotorTankBase;
	bool isTwoMotorTankBase;

}Base;

typedef struct PID {
	bool isDatalogEnabled;
	bool isEnabled;
	bool atTarget;
	tSensors inputSensor;
	tMotor motorOutput;
	tMotor velocityMotorInput;
	int maxOutput;
	int minOutput;
	float error;
	float previousError;
	float integral;
	float derivative;
	float kP;
	float kI;
	float kD;
	float setpoint;
	float tolerance;
	float output;
	float mySensorScale;
	float integralLimit;

} PID;

//Create your structs here
struct PID rotatePID;
struct PID basePID;
struct PID gyroPID;
struct PID leftBasePID;
struct PID rightBasePID;
struct PID chainPID;
struct PID armPID;
struct PID leftClawPID;
struct PID rightClawPID;
struct Base myRobotBase;

int MOTOR_DEFAULT_SLEW_RATE = 15;      // Default will cause 375mS from full fwd to rev

#include "lcdLib.c"
#include "LCD_API.c"
#include "Vex_Competition_Includes_Copy1.c"
#include "SlewRate_API.c"
#include "PID_API.c"
#include "Movement_API.c"
#include "Controller_API.c"
