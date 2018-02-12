/*/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
Driver Functions
*//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool manualA = true;
bool manualC = true;

int power = 127;
int intakeToggle = 1;

void manualArm(TVexJoysticks button1, TVexJoysticks button2) {

	if (vexRT[button1] == 1) {


			manualA = true;
			motor[LiftLeftTop]=power;
			motor[LiftRightTop]=power;
			motor[LiftLeftBottom]=power;
			motor[LiftRightBottom]=power;

}
	else if (vexRT[button2] == 1) {
			motor[LiftLeftTop]=-power;
			motor[LiftRightTop]=-power;
			motor[LiftLeftBottom]=-power;
			motor[LiftRightBottom]=-power;
	}

	else {
		motor[LiftLeftTop]=0;
		motor[LiftRightTop]=0;
		motor[LiftLeftBottom]=0;
		motor[LiftRightBottom]=0;
	}
}

void manualWheel(TVexJoysticks Button1, TVexJoysticks Button2) {

if(vexRT[Button1] == 1) {
	motor[Wheel] = 127;

}

else if(vexRT[Button2] == 1) {
	motor[Wheel] = -127;

}
else {
	motor[Wheel] = 0;

}
}
void setArmSetpoint(TVexJoysticks myButton, int position) {
	if (vexRT[myButton] == 1) {
		manualA = false;
		setpoint(armPID, position);
	}
}

void setChainSetpoint(TVexJoysticks myButton, int position) {
	if (vexRT[myButton] == 1) {

		setpoint(chainPID, position);
	}
}

void PIDChain() {
	int armPower = PIDControl(chainPID);

	if (!PIDatSetpoint(chainPID)) {
			motor[BarLeft]=-armPower;
			motor[BarRight]=-armPower;
	}
}


void PIDArm() {
	int armPower = PIDControl(armPID);

	if (!PIDatSetpoint(armPID)) {
			motor[LiftLeftTop]=armPower;
			motor[LiftRightTop]=-armPower;
			motor[LiftLeftBottom]=-armPower;
			motor[LiftRightBottom]=armPower;
	}
}


void setClawSetpoint(TVexJoysticks myButton, int leftPosition, int rightPosition) {
	if (vexRT[myButton] == 1) {
		leftClawPID.kI = leftTempKi;
		rightClawPID.kI = rightTempKi;
		manualC = false;
		setpoint(leftClawPID, leftPosition);
		setpoint(rightClawPID, rightPosition);
	}
}

void PIDClaw() {
	int leftClawPIDPower = PIDControl(leftClawPID);
	int rightClawPIDPower = PIDControl(rightClawPID);

	if (!PIDatSetpoint(leftClawPID)) {

		motor[Intake] = leftClawPIDPower;
	}

}

void switchManual(TVexJoysticks myButton) {
	if (vexRT[myButton] == 1) {
		if (manualA) {
			manualA = false;
		}
		else {
			manualA = true;
		}
		if (manualC) {
			manualC = false;
		}
		else {
			manualC = true;
		}
		while(vexRT[myButton] == 1) {
		}
	}
}
void liftObject(TVexJoysticks myButton) {

	if (vexRT[myButton] == 1) {
		if (manualA) {

			manualA = false;
			armPID.setpoint = 500;
		}
		else {
			manualA = true;
		}
		while(vexRT[myButton] == 1) {
		}
	}
}



/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void arcadeDrive() {
		int drive_l_motor;
    int drive_r_motor;


      // get js values
      drive_l_motor = vexRT[Ch3] + vexRT[Ch1];
      drive_r_motor = vexRT[Ch3] - vexRT[Ch1];

      // deadband
      if( abs(drive_l_motor) < 15 )
        drive_l_motor = 0;
      if( abs(drive_r_motor) < 15 )
        drive_r_motor = 0;

      // send to motorReqs
     	motor[BaseLeft] = drive_l_motor;
      motor[BaseRight] = drive_r_motor;

      wait1Msec(20);
    }



void BarManual(TVexJoysticks open, TVexJoysticks close) {


	///Close///
		if(vexRT[close] == 1)  {
			motor[BarLeft] = 115;
			motor[BarRight] = 115;


	}
	///Open///
		else if(vexRT[open] == 1)  {
			motor[BarLeft] = -115;
			motor[BarRight] = -115;


	}

		else {
			motor[BarLeft] = 0;
			motor[BarRight] = 0;

	}

}

void ClawManual (TVexJoysticks up, TVexJoysticks down)  {
		if(vexRT[up] == 1)  {
			motor[Intake] = 75;
			intakeToggle = 1;
			}
	///Open///
		else if(vexRT[down] == 1)  {
			motor[Intake] = -75;
			intakeToggle = 0;
	}

		else {

			if(intakeToggle == 0)  {
				motor[Intake] = -25;
		}

			else if(intakeToggle == 1)  {
				motor[Intake] = 0;
		}
	}
}



/*
void Lift(TVexJoysticks Up, TVexJoysticks down) {

	///Up///
		if(vexRT[Up] == 1) {
			if(SensorValue[ArmPotent] <1000) {
				motor[port3] = 127;
				motor[port4] = 127;
				motor[port7] = 127;
				motor[port8] = 127;
		}
			else {
				motor[port3] = 0;
				motor[port4] = 0;
				motor[port7] = 0;
				motor[port8] = 0;
			}
	}
	///Down///
		else if(vexRT[Down] == 1) {
		if(SensorValue[ArmPotent] >= 0) {
				motor[port3] = -127;
				motor[port4] = -127;
				motor[port7] = -127;
				motor[port8] = -127;
		}
			else {
				motor[port3] = 0;
				motor[port4] = 0;
				motor[port7] = 0;
				motor[port8] = 0;
			}
		}
		else {
			motor[port3] = 0;
			motor[port4] = 0;
			motor[port7] = 0;
			motor[port8] = 0;
	}
}

*/
/*/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
Driver Setups
*//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void singleControl() {

arcadeDrive();


	if (manualA) {
		manualArm(Btn6D, Btn6U);  //The Manual Control for the Robot's Lift


	}
	else {

			if(vexRT[Btn8U] == 1){
			armForMobile = armForMobile + 50;
			setArmSetpoint(Btn8U, armForMobile);
	}
		else if(vexRT[Btn8L] == 1) {
			armForMobile = armForMobile - 50;
			setArmSetpoint(Btn8L, armForMobile);
}

			setArmSetpoint(Btn6D, armAtFloor);
			setArmSetpoint(Btn6U, armAtStation);
			PIDArm();
}


BarManual(Btn7U, Btn7D);
manualWheel(Btn5U, Btn5D);
ClawManual(Btn7L, Btn7R);
switchManual(Btn8D);
//PIDChain();
//setChainSetpoint(Btn7U, 0);
}



void partnerControl() {
arcadeDrive(); //Lucas



	if (manualA) { //Cole
		manualArm(Btn5DXmtr2, Btn5UXmtr2);  //The Manual Control for the Robot's Lift
	}
	else {

			if(vexRT[Btn8UXmtr2] == 1){
			armForMobile = armForMobile + 50;
			setArmSetpoint(Btn8UXmtr2, armForMobile);
	}
		else if(vexRT[Btn8LXmtr2] == 1) {
			armForMobile = armForMobile - 50;
			setArmSetpoint(Btn8LXmtr2, armForMobile);
}

			setArmSetpoint(Btn6DXmtr2, armAtFloor);
			setArmSetpoint(Btn6UXmtr2, armAtStation);
			PIDArm();
}

BarManual(Btn6UXmtr2, Btn6DXmtr2);//Cole
manualWheel(Btn5U, Btn5D);//Lucas
ClawManual(Btn7UXmtr2, Btn7DXmtr2);//Cole

	//switchManual(Btn8DXmtr2);//Cole
}
