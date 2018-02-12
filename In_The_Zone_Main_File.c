#pragma config(UART_Usage, UART1, uartVEXLCD, baudRate19200, IOPins, None, None)
#pragma config(Sensor, in1,    ,               sensorGyro)
#pragma config(Sensor, in2,    BATTERY_2_PORT, sensorAnalog)
#pragma config(Sensor, in3,    ChainPotent,    sensorPotentiometer)
#pragma config(Sensor, in4,    Doesntwork,     sensorNone)
#pragma config(Sensor, in5,    ArmPotent,      sensorPotentiometer)
#pragma config(Sensor, in6,    Doesntwork,     sensorNone)
#pragma config(Sensor, in7,    Gyro,           sensorGyro)
#pragma config(Sensor, in8,    LCDPotent,      sensorPotentiometer)
#pragma config(Sensor, dgtl2,  ChainEncoder,   sensorQuadEncoder)
#pragma config(Sensor, dgtl5,  LCDEncoder,     sensorQuadEncoder)
#pragma config(Sensor, dgtl7,  rightBaseEncoder, sensorQuadEncoder)
#pragma config(Sensor, dgtl9,  lcdButton,      sensorDigitalIn)
#pragma config(Sensor, dgtl10, manualLight,    sensorDigitalOut)
#pragma config(Sensor, dgtl11, leftBaseEncoder, sensorQuadEncoder)
#pragma config(Motor,  port1,           BarLeft,       tmotorVex393_HBridge, openLoop)
#pragma config(Motor,  port2,           BaseLeft,      tmotorVex393HighSpeed_MC29, openLoop, reversed)
#pragma config(Motor,  port3,           LiftLeftBottom, tmotorVex393_MC29, openLoop, reversed)
#pragma config(Motor,  port4,           LiftLeftTop,   tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port5,           Intake,        tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port6,           Wheel,         tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port7,           LiftRightTop,  tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port8,           LiftRightBottom, tmotorVex393_MC29, openLoop, reversed)
#pragma config(Motor,  port9,           BaseRight,     tmotorVex393HighSpeed_MC29, openLoop, reversed)
#pragma config(Motor,  port10,          BarRight,      tmotorVex393_HBridge, openLoop, reversed)
//*!!Code automatically generated by 'ROBOTC' configuration wizard               !!*//

int factor;
#include "RealSteelLibrary.c"

//Global Variables


//Global Constants

//Game include files
const int armAtFloor = 0;
//const int leftClawClosed = 1700;
//const int rightClawClosed = 1000;

int armForMobile = 200;

//const int armPreload = 400;
int armAtStation = 600;

//const int leftClaw45 = leftClawClosed + 1000;
//const int rightClaw45 = rightClawClosed - 500;

float leftTempKi;
float rightTempKi;

#include "In_The_Zone_Driver.c"
#include "In_The_Zone_LCD.c"
#include "In_The_Zone_Driver.c"
#include "In_The_Zone_Auto.c"

//Debugger Windows
#pragma DebuggerWindows("VexLCD")
#pragma DebuggerWindows("debugStream")
#pragma DebuggerWindows("Motors")
#pragma DebuggerWindows("Sensors")
#pragma DebuggerWindows("taskStatus")

void robotMap() {
	myRobotBase.frontLeftBaseMotor = BaseLeft;
	myRobotBase.frontRightBaseMotor = BaseRight;
	myRobotBase.encoderTickCount = 360;
	myRobotBase.radius = 2;
	myRobotBase.myGyro = Gyro;

	myRobotBase.isHolonomicBase = false;
	myRobotBase.isTwoMotorTankBase = true;
	myRobotBase.isFourMotorTankBase = false;
}

/*---------------------------------------------------------------------------*/
/*                          Pre-Autonomous Functions                         */
/*                                                                           */
/*  You may want to perform some actions before the competition starts.      */
/*  Do them in the following function.  You must return from this function   */
/*  or the autonomous and usercontrol tasks will not be started.  This       */
/*  function is only called once after the cortex has been powered on and    */
/*  not every time that the robot is disabled.                               */
/*---------------------------------------------------------------------------*/
void pre_auton()
{
	robotMap();
	//clear and reset LCD 1 on UART1
	clearDebugStream();
	clearLCDLine(0);
	clearLCDLine(1);
	bLCDBacklight = true;

	//clear and reset LCD 2 on UART2
	vexLcdInit(UART1);
	vexLcdClearLine(0);
	vexLcdClearLine(1);
	vexLcdBacklight(1);

	bStopTasksBetweenModes = false;

	displayLCDCenteredString(0, "Calibrating Gyro");

	SensorType[in1] = sensorNone;
	wait1Msec(1000);
	//Completely clear out any previous sensor readings by setting the port to "sensorNone"
	//Reconfigure Analog Port 2 as a Gyro sensor and allow time for ROBOTC to calibrate it
	SensorType[in1] = sensorGyro;
	wait1Msec(2000);
	//clear and reset everything

	clearLCDLine(0);
	//startTask(LCD);
	//startTask(MotorSlewRateTask);

	//PIDInitializeController(pPID, inputSensor, setpoint, maxOutput, minOutput, mySensorScale, integralLimit, kP, kI, kD);

	//PIDInitializeController(basePID, rightBaseEncoder, 0, 100, 30, 1, 0, .1, 0, 0);

	//PIDInitializeController(leftBasePID, leftBaseEncoder, 0, 100, 20, 1, 0, .425, 0 ,.1);
	//PIDInitializeController(rightBasePID, rightBaseEncoder, 0, 100, 20, 1, 0, .425, 0, .1);

	//PIDInitializeController(gyroPID, rightBaseEncoder, 0, 100, 30, -1, 0, .4, 0, 0);

	//PIDInitializeController(armPID, ArmPotent, 700, 127, 1, 1, 0, .25, 0, 0);

	//PIDInitializeController(chainPID, ChainEncoder, 0, 100, 1, 1, 0, .1, 0, 0);

	leftTempKi = leftClawPID.kI;
	rightTempKi = rightClawPID.kI;

	myRobotBase.leftCompensation = 1;
	myRobotBase.rightCompensation = 1;

	chainPID.tolerance = 40;
	leftClawPID.tolerance = 40;
	rightClawPID.tolerance = 40;
	chainPID.tolerance = 40;
	gyroPID.tolerance = 10;

	gyroPID.isDatalogEnabled = true;

}

task autonomous()
{

	switch (AutonChoice) {
	case 0:
		Nothing();
		break;
	case 1:
		BlueRight();
		break;
	case 2:
		BlueLeft();
		break;
	case 3:
		RedRight();
		break;
	case 4:
		RedLeft();
		break;
	case 5:
		Skills();
		break;
	case 6:
		BlueRightfivepoint();
		break;
	case 7:
		BlueLeftfivepoint();
		break;
	case 8:
		RedRightfivepoint();
		break;
	case 9:
		RedLeftfivepoint();
		break;
	}
}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              User Control Task                            */
/*                                                                           */
/*  This task is used to control your robot during the user control phase of */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/

task usercontrol()
{
	//stopTask(LCD);
	displayLCDString(0, 0, "                ");
	displayLCDString(1, 0, "                ");
	vexLcdSet(0, "                ");
	vexLcdSet(1, "                ");

	//MOTOR_DEFAULT_SLEW_RATE = 10;
	while (true)
	{
		if (SensorValue[lcdButton] == 0) {
			//startTask(LCD);
			while(SensorValue[lcdButton] == 0) {
			}
		}
		if (isPartner) {

			switch(whichDriver) {
			case 0:
				partnerControl();

				break;
			case 1:
				break;
			case 2:
				break;
			case 3:
				break;
			}
		}
		else {
			partnerControl();
		}

		wait1Msec(20);
	}
}