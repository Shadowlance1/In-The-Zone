bool isTimeout = false;
int timeout;
int offset = 0;

void setTimeout() {
	isTimeout = false;
	clearTimer(T1);
}

void getTimeout() {
	if (time1[T1] < timeout) {
		isTimeout = false;
	}
	else {
		isTimeout = true;
	}
}

float convertInchesToEncoder(float inputDistanceInInches) {
	return (inputDistanceInInches * myRobotBase.encoderTickCount) / (2 * PI* myRobotBase.radius);
}

float getAngle(float angle) {
	return (angle/100);
}

void drive(int magnitude, int curve) {

}

void baseDatalog() {
	if (basePID.isDatalogEnabled) {
		datalogDataGroupStart();
		datalogAddValue(0, basePID.setpoint);
		datalogAddValue(1, basePID.error);
		datalogAddValue(2, basePID.derivative);
		datalogAddValue(3, basePID.output);
		datalogAddValue(4, gyroPID.setpoint);
		datalogAddValue(5, gyroPID.error);
		datalogAddValue(6, gyroPID.derivative);
		datalogAddValue(7, gyroPID.output);
		datalogDataGroupEnd();

	}
}

void gyroDatalog() {
	if (gyroPID.isDatalogEnabled) {
		datalogDataGroupStart();
		datalogAddValue(4, gyroPID.setpoint);
		datalogAddValue(5, gyroPID.error);
		datalogAddValue(6, gyroPID.derivative);
		datalogAddValue(7, gyroPID.output);
		datalogDataGroupEnd();
	}
}

void moveForward(int distance, int power, float myTimeout) {
	basePID.setpoint = convertInchesToEncoder(distance);
	basePID.maxOutput = power;
	basePID.mySensorScale = -1;
	timeout = myTimeout * 1000;
	SensorValue[basePID.inputSensor] = 0;
	SensorValue[myRobotBase.myGyro] = 0;
	basePID.tolerance = 15;

	bool atSetpoint = false;
	setTimeout();

	if (myRobotBase.isHolonomicBase) {
		while ((!atSetpoint && bIfiAutonomousMode) && !isTimeout) {

		}
	}
	else if (myRobotBase.isFourMotorTankBase) {
		while ((!atSetpoint && bIfiAutonomousMode) && !isTimeout) {

			float basePIDOutput = PIDControl(basePID);
			//float gyroPIDOutput = PIDControl(gyroPID);

			baseDatalog();

			motorReq[myRobotBase.frontLeftBaseMotor] = basePIDOutput * myRobotBase.leftCompensation;
			motorReq[myRobotBase.backLeftBaseMotor] = basePIDOutput * myRobotBase.leftCompensation;
			motorReq[myRobotBase.frontRightBaseMotor] = basePIDOutput * myRobotBase.rightCompensation;
			motorReq[myRobotBase.backRightBaseMotor] = basePIDOutput * myRobotBase.rightCompensation;

			wait1Msec(20);
			atSetpoint = PIDatSetpoint(basePID);
			getTimeout();
		}
		motorReq[myRobotBase.frontLeftBaseMotor] = 0;
		motorReq[myRobotBase.backLeftBaseMotor] = 0;
		motorReq[myRobotBase.frontRightBaseMotor] = 0;
		motorReq[myRobotBase.backRightBaseMotor] = 0;

	}
	else if (myRobotBase.isTwoMotorTankBase) {
		while ((!atSetpoint && bIfiAutonomousMode) && !isTimeout) {

			float basePIDOutput = PIDControl(basePID);

			baseDatalog();

				motorReq[myRobotBase.frontLeftBaseMotor] = basePIDOutput * myRobotBase.leftCompensation;
				motorReq[myRobotBase.frontRightBaseMotor] = basePIDOutput * myRobotBase.rightCompensation;


			wait1Msec(20);
			atSetpoint = PIDatSetpoint(basePID);
			getTimeout();
		}
		motorReq[myRobotBase.frontLeftBaseMotor] = 0;
		motorReq[myRobotBase.frontRightBaseMotor] = 0;

	}
	else {
		writeDebugStreamLine("No drive set");
	}
}

void moveForward(int distance, int power) {
	basePID.setpoint = convertInchesToEncoder(distance);
	basePID.maxOutput = power;
	basePID.mySensorScale = 1;
	SensorValue[basePID.inputSensor] = 0;
	SensorValue[myRobotBase.myGyro] = 0;
	basePID.tolerance = 15;

	bool atSetpoint = false;
	setTimeout();

	if (myRobotBase.isHolonomicBase) {
		while ((!atSetpoint && bIfiAutonomousMode) && !isTimeout) {

		}
	}
	else if (myRobotBase.isFourMotorTankBase) {
		while ((!atSetpoint && bIfiAutonomousMode) && !isTimeout) {

			float basePIDOutput = PIDControl(basePID);
			//float gyroPIDOutput = PIDControl(gyroPID);

			baseDatalog();

			motorReq[myRobotBase.frontLeftBaseMotor] = basePIDOutput * myRobotBase.leftCompensation;
			motorReq[myRobotBase.backLeftBaseMotor] = basePIDOutput * myRobotBase.leftCompensation;
			motorReq[myRobotBase.frontRightBaseMotor] = basePIDOutput * myRobotBase.rightCompensation;
			motorReq[myRobotBase.backRightBaseMotor] = basePIDOutput * myRobotBase.rightCompensation;

			wait1Msec(20);
			atSetpoint = PIDatSetpoint(basePID);
		}
		motorReq[myRobotBase.frontLeftBaseMotor] = 0;
		motorReq[myRobotBase.backLeftBaseMotor] = 0;
		motorReq[myRobotBase.frontRightBaseMotor] = 0;
		motorReq[myRobotBase.backRightBaseMotor] = 0;

	}
	else if (myRobotBase.isTwoMotorTankBase) {
		while ((!atSetpoint && bIfiAutonomousMode) && !isTimeout) {

			float basePIDOutput = PIDControl(basePID);
			float gyroPIDOutput = PIDControl(gyroPID);
			baseDatalog();

			motorReq[myRobotBase.frontLeftBaseMotor] = basePIDOutput * myRobotBase.leftCompensation;
			motorReq[myRobotBase.frontRightBaseMotor] = basePIDOutput * myRobotBase.rightCompensation;


			wait1Msec(20);
			atSetpoint = PIDatSetpoint(basePID);
		}
		motorReq[myRobotBase.frontLeftBaseMotor] = 0;
		motorReq[myRobotBase.frontRightBaseMotor] = 0;

	}
	else {
		writeDebugStreamLine("No drive set");
	}
}

void moveBackward(int distance, int power, int myTimeout) {
	basePID.setpoint = convertInchesToEncoder(distance);
	basePID.maxOutput = power;
	basePID.mySensorScale = 1;
	timeout = myTimeout * 1000;
	SensorValue[basePID.inputSensor] = 0;
	SensorValue[myRobotBase.myGyro] = 0;
	basePID.tolerance = 15;

	bool atSetpoint = false;
	setTimeout();

	if (myRobotBase.isHolonomicBase) {
		while ((!basePID.atTarget && bIfiAutonomousMode) && !isTimeout) {

		}
	}
	else if (myRobotBase.isFourMotorTankBase) {
		while ((!atSetpoint && bIfiAutonomousMode) && !isTimeout) {

			float basePIDOutput = PIDControl(basePID);
			//float gyroPIDOutput = PIDControl(gyroPID);

			baseDatalog();

			motorReq[myRobotBase.frontLeftBaseMotor] = -basePIDOutput * myRobotBase.leftCompensation;
			motorReq[myRobotBase.backLeftBaseMotor] = -basePIDOutput * myRobotBase.leftCompensation;
			motorReq[myRobotBase.frontRightBaseMotor] = -basePIDOutput * myRobotBase.rightCompensation;
			motorReq[myRobotBase.backRightBaseMotor] = -basePIDOutput * myRobotBase.rightCompensation;

			wait1Msec(20);
			atSetpoint = PIDatSetpoint(basePID);
			getTimeout();
		}
		motorReq[myRobotBase.frontLeftBaseMotor] = 0;
		motorReq[myRobotBase.backLeftBaseMotor] = 0;
		motorReq[myRobotBase.frontRightBaseMotor] = 0;
		motorReq[myRobotBase.backRightBaseMotor] = 0;

	}
	else if (myRobotBase.isTwoMotorTankBase) {
		while ((!atSetpoint && bIfiAutonomousMode) && !isTimeout) {

			float basePIDOutput = PIDControl(basePID);
			float gyroPIDOutput = PIDControl(gyroPID);

			baseDatalog();
			motorReq[myRobotBase.frontLeftBaseMotor] = -basePIDOutput * myRobotBase.leftCompensation;
			motorReq[myRobotBase.frontRightBaseMotor] = -basePIDOutput * myRobotBase.rightCompensation;



			wait1Msec(20);
			atSetpoint = PIDatSetpoint(basePID);
			getTimeout();
		}
		motorReq[myRobotBase.frontLeftBaseMotor] = 0;
		motorReq[myRobotBase.frontRightBaseMotor] = 0;

	}
	else {
		writeDebugStreamLine("No drive set");
	}
}

void moveBackward(int distance, int power) {
	basePID.setpoint = convertInchesToEncoder(distance);
	basePID.maxOutput = power;
	basePID.mySensorScale = 1;
	SensorValue[basePID.inputSensor] = 0;
	SensorValue[myRobotBase.myGyro] = 0;
	basePID.tolerance = 15;

	bool atSetpoint = false;
	setTimeout();

	if (myRobotBase.isHolonomicBase) {
		while ((!basePID.atTarget && bIfiAutonomousMode) && !isTimeout) {

		}
	}
	else if (myRobotBase.isFourMotorTankBase) {
		while ((!atSetpoint && bIfiAutonomousMode) && !isTimeout) {

			float basePIDOutput = PIDControl(basePID);
			//float gyroPIDOutput = PIDControl(gyroPID);

			baseDatalog();

			motorReq[myRobotBase.frontLeftBaseMotor] = -basePIDOutput * myRobotBase.leftCompensation;
			motorReq[myRobotBase.backLeftBaseMotor] = -basePIDOutput * myRobotBase.leftCompensation;
			motorReq[myRobotBase.frontRightBaseMotor] = -basePIDOutput * myRobotBase.rightCompensation;
			motorReq[myRobotBase.backRightBaseMotor] = -basePIDOutput * myRobotBase.rightCompensation;

			wait1Msec(20);
			atSetpoint = PIDatSetpoint(basePID);
		}
		motorReq[myRobotBase.frontLeftBaseMotor] = 0;
		motorReq[myRobotBase.backLeftBaseMotor] = 0;
		motorReq[myRobotBase.frontRightBaseMotor] = 0;
		motorReq[myRobotBase.backRightBaseMotor] = 0;

	}
	else if (myRobotBase.isTwoMotorTankBase) {
		while ((!atSetpoint && bIfiAutonomousMode) && !isTimeout) {

			float basePIDOutput = PIDControl(basePID);
			float gyroPIDOutput = PIDControl(gyroPID);

			baseDatalog();
			motorReq[myRobotBase.frontLeftBaseMotor] = -basePIDOutput * myRobotBase.leftCompensation;
			motorReq[myRobotBase.frontRightBaseMotor] = -basePIDOutput * myRobotBase.rightCompensation;



			wait1Msec(20);
			atSetpoint = PIDatSetpoint(basePID);
		}
		motorReq[myRobotBase.frontLeftBaseMotor] = 0;
		motorReq[myRobotBase.frontRightBaseMotor] = 0;

	}
	else {
		writeDebugStreamLine("No drive set");
	}
}

void moveLeft(int distnace, int power) {
	if (myRobotBase.isHolonomicBase) {

	}
	else {

	}
}

void moveRight(int distance, int power) {
	if (myRobotBase.isHolonomicBase) {

	}
	else {

	}
}

void turnLeft(int angle, int power) {
	gyroPID.error = 0;
	gyroPID.setpoint = SensorValue[myRobotBase.myGyro] + (angle*10);
	gyroPID.maxOutput = power;


	bool atSetpoint = false;

	if (myRobotBase.isHolonomicBase) {
		while (!basePID.atTarget && bIfiAutonomousMode) {

		}
	}
	else if (myRobotBase.isFourMotorTankBase) {
		while (!atSetpoint && bIfiAutonomousMode) {

			float gyroPIDOutput = PIDControl(gyroPID);

			gyroDatalog();

			motorReq[myRobotBase.frontLeftBaseMotor] = -gyroPIDOutput;
			motorReq[myRobotBase.backLeftBaseMotor] = gyroPIDOutput;
			motorReq[myRobotBase.frontRightBaseMotor] = gyroPIDOutput;
			motorReq[myRobotBase.backRightBaseMotor] = -gyroPIDOutput;



			wait1Msec(20);
			atSetpoint = PIDatSetpoint(gyroPID);
		}
		motorReq[myRobotBase.frontLeftBaseMotor] = 0;
		motorReq[myRobotBase.backLeftBaseMotor] = 0;
		motorReq[myRobotBase.frontRightBaseMotor] = 0;
		motorReq[myRobotBase.backRightBaseMotor] = 0;

	}
	else if (myRobotBase.isTwoMotorTankBase) {
		while (!atSetpoint && bIfiAutonomousMode) {

			float gyroPIDOutput = PIDControl(gyroPID);

			gyroDatalog();

			motorReq[myRobotBase.frontLeftBaseMotor] = -gyroPIDOutput;
			motorReq[myRobotBase.frontRightBaseMotor] = gyroPIDOutput;



			wait1Msec(20);
			atSetpoint = PIDatSetpoint(gyroPID);
		}
		motorReq[myRobotBase.frontLeftBaseMotor] = 0;
		motorReq[myRobotBase.frontRightBaseMotor] = 0;

	}
	else {
		writeDebugStreamLine("No drive set");
	}
}

void turnLeft(int angle, int power, float myTimeout) {
	gyroPID.error = 0;
	gyroPID.setpoint = SensorValue[myRobotBase.myGyro] + (angle * 10);
	gyroPID.maxOutput = power;
	timeout = 2 * 1000;


	bool atSetpoint = false;
	setTimeout();

	if (myRobotBase.isHolonomicBase) {
		while ((!basePID.atTarget && bIfiAutonomousMode) && !isTimeout) {

		}
	}
	else if (myRobotBase.isFourMotorTankBase) {
		while ((!atSetpoint && bIfiAutonomousMode) && !isTimeout) {

			float gyroPIDOutput = PIDControl(gyroPID);

			gyroDatalog();

			motorReq[myRobotBase.frontLeftBaseMotor] = gyroPIDOutput;
			motorReq[myRobotBase.backLeftBaseMotor] = -gyroPIDOutput;
			motorReq[myRobotBase.frontRightBaseMotor] = -gyroPIDOutput;
			motorReq[myRobotBase.backRightBaseMotor] = gyroPIDOutput;



			wait1Msec(20);
			atSetpoint = PIDatSetpoint(gyroPID);
			getTimeout();
		}
		motorReq[myRobotBase.frontLeftBaseMotor] = 0;
		motorReq[myRobotBase.backLeftBaseMotor] = 0;
		motorReq[myRobotBase.frontRightBaseMotor] = 0;
		motorReq[myRobotBase.backRightBaseMotor] = 0;

	}
	else if (myRobotBase.isTwoMotorTankBase) {
		while ((!atSetpoint && bIfiAutonomousMode) && !isTimeout) {

			float gyroPIDOutput = PIDControl(gyroPID);

			gyroDatalog();

			motorReq[myRobotBase.frontLeftBaseMotor] = -gyroPIDOutput;
			motorReq[myRobotBase.frontRightBaseMotor] = gyroPIDOutput;



			wait1Msec(20);
			atSetpoint = PIDatSetpoint(gyroPID);
			getTimeout();
		}
		motorReq[myRobotBase.frontLeftBaseMotor] = 0;
		motorReq[myRobotBase.frontRightBaseMotor] = 0;

	}
	else {
		writeDebugStreamLine("No drive set");
	}
}

void turnRightRelative(int angle, int power) {
	gyroPID.error = 0;
	gyroPID.setpoint = SensorValue[myRobotBase.myGyro] + (angle * 10);
	gyroPID.maxOutput = power;


	bool atSetpoint = false;

	if (myRobotBase.isHolonomicBase) {
		while (!basePID.atTarget && bIfiAutonomousMode) {

		}
	}
	else if (myRobotBase.isFourMotorTankBase) {
		while (!atSetpoint && bIfiAutonomousMode) {

			float gyroPIDOutput = PIDControl(gyroPID);

			gyroDatalog();

			motorReq[myRobotBase.frontLeftBaseMotor] = -gyroPIDOutput;
			motorReq[myRobotBase.backLeftBaseMotor] = -gyroPIDOutput;
			motorReq[myRobotBase.frontRightBaseMotor] = gyroPIDOutput;
			motorReq[myRobotBase.backRightBaseMotor] = gyroPIDOutput;



			wait1Msec(20);
			atSetpoint = PIDatSetpoint(gyroPID);
		}
		motorReq[myRobotBase.frontLeftBaseMotor] = 0;
		motorReq[myRobotBase.backLeftBaseMotor] = 0;
		motorReq[myRobotBase.frontRightBaseMotor] = 0;
		motorReq[myRobotBase.backRightBaseMotor] = 0;

	}
	else if (myRobotBase.isTwoMotorTankBase) {
		while (!atSetpoint && bIfiAutonomousMode) {

			float gyroPIDOutput = PIDControl(gyroPID);

			gyroDatalog();

			motorReq[myRobotBase.frontLeftBaseMotor] = gyroPIDOutput;
			motorReq[myRobotBase.frontRightBaseMotor] = -gyroPIDOutput;



			wait1Msec(20);
			atSetpoint = PIDatSetpoint(gyroPID);
		}
		motorReq[myRobotBase.frontLeftBaseMotor] = 0;
		motorReq[myRobotBase.frontRightBaseMotor] = 0;

	}
	else {

	}
}

void turnRightRelative(int angle, int power, float myTimeout) {
	gyroPID.error = 0;
	gyroPID.setpoint = SensorValue[myRobotBase.myGyro] - (angle*10);
	gyroPID.maxOutput = power;
	timeout = myTimeout * 1000;
	gyroPID.tolerance = 25;

	bool atSetpoint = false;
	setTimeout();

	if (myRobotBase.isHolonomicBase) {
		while ((!basePID.atTarget && bIfiAutonomousMode) && !isTimeout) {

		}
	}
	else if (myRobotBase.isFourMotorTankBase) {
		while ((!atSetpoint && bIfiAutonomousMode) && !isTimeout) {

			float gyroPIDOutput = PIDControl(gyroPID);

			gyroDatalog();

			motorReq[myRobotBase.frontLeftBaseMotor] = -gyroPIDOutput;
			motorReq[myRobotBase.backLeftBaseMotor] = -gyroPIDOutput;
			motorReq[myRobotBase.frontRightBaseMotor] = gyroPIDOutput;
			motorReq[myRobotBase.backRightBaseMotor] = gyroPIDOutput;



			wait1Msec(20);
			atSetpoint = PIDatSetpoint(gyroPID);
			getTimeout();
		}
		motorReq[myRobotBase.frontLeftBaseMotor] = 0;
		motorReq[myRobotBase.backLeftBaseMotor] = 0;
		motorReq[myRobotBase.frontRightBaseMotor] = 0;
		motorReq[myRobotBase.backRightBaseMotor] = 0;

	}
	else if (myRobotBase.isTwoMotorTankBase) {
		while ((!atSetpoint && bIfiAutonomousMode) && !isTimeout) {

			float gyroPIDOutput = PIDControl(gyroPID);

			gyroDatalog();

			motorReq[myRobotBase.frontLeftBaseMotor] = gyroPIDOutput;
			motorReq[myRobotBase.frontRightBaseMotor] = -gyroPIDOutput;



			wait1Msec(20);
			atSetpoint = PIDatSetpoint(gyroPID);
			getTimeout();
		}
		motorReq[myRobotBase.frontLeftBaseMotor] = 0;
		motorReq[myRobotBase.frontRightBaseMotor] = 0;

	}
	else {

	}
}

void turnToAngle(int angle, int power) {
	gyroPID.error = 0;
	gyroPID.setpoint = (angle * 10) - (offset * 10);
	gyroPID.maxOutput = power;
	gyroPID.tolerance = 20;
	bool atSetpoint = false;


	if (myRobotBase.isHolonomicBase) {
		while (!basePID.atTarget && bIfiAutonomousMode) {

		}
	}
	else if (myRobotBase.isFourMotorTankBase) {
		while (!atSetpoint && bIfiAutonomousMode) {

			float gyroPIDOutput = PIDControl(gyroPID);

			gyroDatalog();

			motorReq[myRobotBase.frontLeftBaseMotor] = gyroPIDOutput;
			motorReq[myRobotBase.backLeftBaseMotor] = gyroPIDOutput;
			motorReq[myRobotBase.frontRightBaseMotor] = -gyroPIDOutput;
			motorReq[myRobotBase.backRightBaseMotor] = -gyroPIDOutput;



			wait1Msec(20);
			atSetpoint = PIDatSetpoint(gyroPID);
		}
		motorReq[myRobotBase.frontLeftBaseMotor] = 0;
		motorReq[myRobotBase.backLeftBaseMotor] = 0;
		motorReq[myRobotBase.frontRightBaseMotor] = 0;
		motorReq[myRobotBase.backRightBaseMotor] = 0;

	}
	else if (myRobotBase.isTwoMotorTankBase) {
		while (!atSetpoint && bIfiAutonomousMode) {

			float gyroPIDOutput = PIDControl(gyroPID);

			gyroDatalog();

			motorReq[myRobotBase.frontLeftBaseMotor] = gyroPIDOutput;
			motorReq[myRobotBase.frontRightBaseMotor] = -gyroPIDOutput;



			wait1Msec(20);
			atSetpoint = PIDatSetpoint(gyroPID);
		}
		motorReq[myRobotBase.frontLeftBaseMotor] = 0;
		motorReq[myRobotBase.frontRightBaseMotor] = 0;

	}
	else {

	}
}

void turnToAngle(int angle, int power, float myTimeout) {
	writeDebugStreamLine("turning");
	gyroPID.error = 0;
	gyroPID.setpoint = (angle * 10) - (offset * 10);
	gyroPID.maxOutput = power;
	gyroPID.tolerance = 20;
	timeout = myTimeout * 1000;

	bool atSetpoint = false;
	setTimeout();

	if (myRobotBase.isHolonomicBase) {
		while ((!basePID.atTarget && bIfiAutonomousMode) && !isTimeout) {

		}
	}
	else if (myRobotBase.isFourMotorTankBase) {
		while ((bIfiAutonomousMode) && !isTimeout) {

			float gyroPIDOutput = PIDControl(gyroPID);

			gyroDatalog();

			motorReq[myRobotBase.frontLeftBaseMotor] = gyroPIDOutput;
			motorReq[myRobotBase.backLeftBaseMotor] = -gyroPIDOutput;
			motorReq[myRobotBase.frontRightBaseMotor] = -gyroPIDOutput;
			motorReq[myRobotBase.backRightBaseMotor] = gyroPIDOutput;



			wait1Msec(20);
			atSetpoint = PIDatSetpoint(gyroPID);
			getTimeout();
		}
		motorReq[myRobotBase.frontLeftBaseMotor] = 0;
		motorReq[myRobotBase.backLeftBaseMotor] = 0;
		motorReq[myRobotBase.frontRightBaseMotor] = 0;
		motorReq[myRobotBase.backRightBaseMotor] = 0;

	}
	else if (myRobotBase.isTwoMotorTankBase) {
		while ((bIfiAutonomousMode) && !isTimeout) {

			float gyroPIDOutput = PIDControl(gyroPID);
			writeDebugStreamLine("output: %d", gyroPIDOutput);
			gyroDatalog();

			motorReq[myRobotBase.frontLeftBaseMotor] = gyroPIDOutput;
			motorReq[myRobotBase.frontRightBaseMotor] = -gyroPIDOutput;

			wait1Msec(20);
			atSetpoint = PIDatSetpoint(gyroPID);
			getTimeout();
		}
		motorReq[myRobotBase.frontLeftBaseMotor] = 0;
		motorReq[myRobotBase.frontRightBaseMotor] = 0;

	}
	else {

	}
}
