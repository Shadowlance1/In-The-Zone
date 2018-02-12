void waitUntilControllerRelease(TVexJoysticks button) {
	while(vexRT[button] == 1) {
		wait1Msec(10);  //Don't hog CPU
	}
}
//code for a holonomic drive. Needs 4 motors to operate. Define motors in myRobotBase struct.
void holonomicDrive(Base myRobotBase, int deadband) {
	int X1;
	int Y1;
	int X2;

	if (vexRT[Ch4] <= deadband && vexRT[Ch4] >= -deadband) {
		X1 = 0;
	}
	else {
		X1 = vexRT[Ch4];
	}

	if (vexRT[Ch3] <= deadband && vexRT[Ch3] >= -deadband) {
		Y1 = 0;
	}
	else {
		Y1 = vexRT[Ch3];
	}

	if (vexRT[Ch1] <= deadband && vexRT[Ch1] >= -deadband) {
		X2 = 0;
	}
	else {
		X2 = vexRT[Ch1];
	}

	// Y component, X component, Rotation
	motorReq[myRobotBase.frontLeftBaseMotor] = Y1 + X1 + X2;
	motorReq[myRobotBase.frontRightBaseMotor] =  Y1 - X1 - X2;
	motorReq[myRobotBase.backLeftBaseMotor] =  Y1 - X1 + X2;
	motorReq[myRobotBase.backRightBaseMotor] = Y1 + X1 - X2;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void fourMotorArcadeDriveControl (int threshold) {
	int X2;
	int Y1;

	if (vexRT[Ch1] < threshold && vexRT[Ch1] > -threshold) {
		Y1 = 0;
	}
	else {
		Y1 = vexRT[Ch1];
	}

	if (vexRT[Ch3] < threshold && vexRT[Ch3] > -threshold) {
		X2 = 0;
	}

	else {
		X2 = -vexRT[Ch3];
	}

	motorReq[myRobotBase.frontRightBaseMotor] = (Y1) + (X2);
	motorReq[myRobotBase.backRightBaseMotor] = (-(Y1) - (X2)) * -1;
	motorReq[myRobotBase.frontLeftBaseMotor] = (Y1) - (X2);
	motorReq[myRobotBase.backLeftBaseMotor] = (-(Y1) + (X2)) * -1;
}

void twoMotorArcadeDriveControl (int threshold) {
	int X2;
	int Y1;

	if (vexRT[Ch1] < threshold && vexRT[Ch1] > -threshold) {
		Y1 = 0;
	}
	else {
		Y1 = vexRT[Ch1];
	}

	if (vexRT[Ch3] < threshold && vexRT[Ch3] > -threshold) {
		X2 = 0;
	}

	else {
		X2 = -vexRT[Ch3];
	}

	motorReq[myRobotBase.frontRightBaseMotor] = (Y1) + (X2);
	motorReq[myRobotBase.frontLeftBaseMotor] = (Y1) - (X2);
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void partnerFourMotorArcadeDriveControl (int threshold) {
	int X2;
	int Y1;

	if (vexRT[Ch3Xmtr2] < threshold && vexRT[Ch3Xmtr2] > -threshold) {
		Y1 = 0;
	}
	else {
		Y1 = vexRT[Ch3Xmtr2];
	}

	if (vexRT[Ch1Xmtr2] < threshold && vexRT[Ch1Xmtr2] > -threshold) {
		X2 = 0;
	}

	else {
		X2 = -vexRT[Ch1Xmtr2];
	}

	motorReq[myRobotBase.frontRightBaseMotor] = (Y1) + (X2);
	motorReq[myRobotBase.backRightBaseMotor] = (Y1) - (X2);
	motorReq[myRobotBase.frontLeftBaseMotor] = (Y1) - (X2);
	motorReq[myRobotBase.backLeftBaseMotor] = (Y1) + (X2);
}

void partnerTwoMotorArcadeDriveControl (int threshold) {
	int X2;
	int Y1;

	if (vexRT[Ch3Xmtr2] < threshold && vexRT[Ch3Xmtr2] > -threshold) {
		Y1 = 0;
	}
	else {
		Y1 = vexRT[Ch3Xmtr2];
	}

	if (vexRT[Ch1Xmtr2] < threshold && vexRT[Ch1Xmtr2] > -threshold) {
		X2 = 0;
	}

	else {
		X2 = -vexRT[Ch1Xmtr2];
	}

	motorReq[myRobotBase.frontRightBaseMotor] = (Y1) + (X2);
	motorReq[myRobotBase.frontLeftBaseMotor] = (Y1) - (X2);
}


void tankFourMotorDrive(Base myRobotBase, int deadband) {
	int Y1;
	int Y2;

	if (vexRT[Ch3] < deadband && vexRT[Ch3] > -deadband) {
		Y1 = 0;
	}
	else {
		Y1 = vexRT[Ch3];
	}

	if (vexRT[Ch2] < deadband && vexRT[Ch2] > -deadband) {
		Y2 = 0;
	}
	else {
		Y2 = vexRT[Ch2];
	}

	motorReq[myRobotBase.frontLeftBaseMotor] = Y1;
	motorReq[myRobotBase.frontRightBaseMotor] = Y1;
	motorReq[myRobotBase.backLeftBaseMotor] = Y2;
	motorReq[myRobotBase.backRightBaseMotor] = Y2;

}

void tankTwoMotorDrive(Base myRobotBase, int deadband) {
	int Y1;
	int Y2;

	if (vexRT[Ch3] < deadband && vexRT[Ch3] > -deadband) {
		Y1 = 0;
	}
	else {
		Y1 = vexRT[Ch3];
	}

	if (vexRT[Ch2] < deadband && vexRT[Ch2] > -deadband) {
		Y2 = 0;
	}
	else {
		Y2 = vexRT[Ch2];
	}

	motorReq[myRobotBase.frontLeftBaseMotor] = Y1;
	motorReq[myRobotBase.frontRightBaseMotor] = Y1;
}

void runMotorOnButtonToggle(tMotor selectedMotor, TVexJoysticks button, bool startingState) {

}

void runMotorOnButtonPress() {

}

void excuteOrder66(TVexJoysticks button, bool functionOn, bool functionOff) {

}

void togglePiston(TVexJoysticks ControlerButton, tSensors thisPiston) {
	if (vexRT[ControlerButton] == 1) {
		switch (SensorValue[thisPiston]) {
		case 0:
			SensorValue[thisPiston] = 1;
			break;
		case 1:
			SensorValue[thisPiston] = 0;
			break;
		}
		while (vexRT[ControlerButton] == 1) {
		}
	}
}

void controlPIDConstants(PID pPID) {

	if (vexRT[Btn7U] == 1) {
		pPID.kP += factor;
		waitUntilControllerRelease(Btn7U);
	}
	else if (vexRT[Btn7L] == 1) {
		pPID.kI += factor;
		waitUntilControllerRelease(Btn7L);
	}
	else if (vexRT[Btn7R] == 1) {
		pPID.kD += factor;
		waitUntilControllerRelease(Btn7R);
	}
	else if (vexRT[Btn7D] == 1) {
		factor += .1;
		waitUntilControllerRelease(Btn7D);
	}

	if (vexRT[Btn8U] == 1) {
		pPID.kP -= factor;
		waitUntilControllerRelease(Btn8U);
	}
	else if (vexRT[Btn8L] == 1) {
		pPID.kD -= factor;
		waitUntilControllerRelease(Btn8L);
	}
	else if (vexRT[Btn8R] == 1) {
		pPID.kI -= factor;
		waitUntilControllerRelease(Btn8R);
	}
	else if (vexRT[Btn8D] == 1) {
		factor -= .1;
		waitUntilControllerRelease(Btn8D);
	}
}
