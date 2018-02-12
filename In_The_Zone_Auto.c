/*/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	Auto Functions to be used for the different setups.
*//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

task autoPID() {
	int armPower;
	int rightClawPower;
	int leftClawPower;

	while(bIfiAutonomousMode) {

		if (armPID.isEnabled) {
			armPower = PIDControl(armPID);
			if (!PIDatSetpoint(armPID)) {
			motor[LiftLeftTop]=armPower;
			motor[LiftRightTop]=-armPower;
			motor[LiftLeftBottom]=-armPower;
			motor[LiftRightBottom]=armPower;
			}
		}

		/*if (leftClawPID.isEnabled) {
			leftClawPower = PIDControl(leftClawPID);
			if (!PIDatSetpoint(leftClawPID)) {
				motorReq[ClawLeft] = -leftClawPower;
			}
		}
		if (rightClawPID.isEnabled) {
			rightClawPower = PIDControl(rightClawPID);
			if (!PIDatSetpoint(rightClawPID)) {
				motorReq[ClawRight] = -rightClawPower;
			}
		}*/
		wait1Msec(20);
	}
	//datalogDataGroupStart();
	//datalogAddValue(0, armPID.setpoint);
	//datalogAddValue(1, armPID.error);
	//datalogAddValue(2, armPower);
	//datalogAddValue(3, leftClawPID.setpoint);
	//datalogAddValue(4, leftClawPID.error);
	//datalogAddValue(5, leftClawPower);
	//datalogAddValue(6, rightClawPID.setpoint);
	//datalogAddValue(7, rightClawPID.error);
	//datalogAddValue(8, rightClawPower);
	//datalogDataGroupEnd();

}


void runArmTime(int power, int time) {
			motor[LiftLeftTop]=power;
			motor[LiftRightTop]=power;
			motor[LiftLeftBottom]=power;
			motor[LiftRightBottom]=power;


	wait1Msec(time);

			motor[LiftLeftTop]=0;
			motor[LiftRightTop]=0;
			motor[LiftLeftBottom]=0;
			motor[LiftRightBottom]=0;
}

void runClawTime(int power, int time) {
	motor[Intake] = power;

	wait1Msec(time);

	motor[Intake] = 0;

}


void runChainTime(int power, int time) {
	motor[BarLeft] = power;
	motor[BarRight] = power;

	wait1Msec(time);

	motor[BarLeft] = 0;
	motor[barRight] = 0;

}

void runForwardTime(int power, int time) {
	motor[BaseLeft] = power;
	motor[BaseRight] = power;

	wait1Msec(time);

	motor[BaseLeft] = 0;
	motor[BaseRight] = 0;

}

void runBackwardTime(int power, int time) {
	motor[BaseLeft] = -power;
	motor[BaseRight] = -power;


	wait1Msec(time);

	motor[BaseLeft] = 0;
	motor[BaseRight] = 0;

}

void runLeftTime(int power, int time) {
	motor[BaseLeft] = -power;
	motor[BaseRight] = power;


	wait1Msec(time);

	motor[BaseLeft] = 0;
	motor[BaseRight] = 0;

}

void runRightTime(int power, int time) {
	motor[BaseLeft] = power;
	motor[BaseRight] = -power;


	wait1Msec(time);

	motor[BaseLeft] = 0;
	motor[BaseRight] = 0;

}

void runSwingLeftTime(int power, int time) {
	motor[BaseLeft] = -50;
	motor[BaseRight] = power;

	wait1Msec(time);

	motor[BaseLeft] = 0;
	motor[BaseRight] = 0;

}

void runSwingRightTime(int power, int time) {
	motor[BaseLeft] = power;
	motor[BaseRight] = -power;


	wait1Msec(time);

	motor[BaseLeft] = 0;
	motor[BaseRight] = 0;

}


void runOpenClawTime(int power, int time) {
	motor[Intake] = -power;


	wait1Msec(time);

	motor[Intake] = 0;

}

void runLowerArmTime(int power, int time) {
			motor[LiftLeftTop]=power;
			motor[LiftRightTop]=power;
			motor[LiftLeftBottom]=power;
			motor[LiftRightBottom]=power;

	wait1Msec(time);

			motor[LiftLeftTop]=0;
			motor[LiftRightTop]=0;
			motor[LiftLeftBottom]=0;
			motor[LiftRightBottom]=0;
}



void runWheelTime(int power, int time) {
	motor[Wheel]=power;
	wait1Msec(time);
	motor[Wheel] = 0;
}


void runForwardEncoder (int encoderCount,int Power){
	SensorValue[leftBaseEncoder] = 0;
	SensorValue[rightBaseEncoder] = 0;


	while (SensorValue[rightBaseEncoder] > encoderCount)
	{ motor(BaseLeft) = Power;
		motor(BaseRight) = Power;
		}
	}

	void runReverseEncoder (int encoderCount,int Power){
	SensorValue[leftBaseEncoder] = 0;
	SensorValue[rightBaseEncoder] = 0;


	while (SensorValue[leftBaseEncoder] < encoderCount)
	{ motor(BaseLeft) = -Power;
		motor(BaseRight) = -Power; }
		}

	void runLeftEncoder(int encoderCount, int Power) {
		SensorValue[leftBaseEncoder] = 0;
    SensorValue[rightBaseEncoder] = 0;


		while (SensorValue[rightBaseEncoder] > encoderCount) {
			motor(BaseLeft)= -Power;
			motor(BaseRight) = Power; }}


		void runRightEncoder(int encoderCount, int Power) {
			SensorValue[leftBaseEncoder] = 0;
    	SensorValue[rightBaseEncoder] = 0;


			while (SensorValue[rightBaseEncoder] < encoderCount) {
				motor(BaseLeft)= Power;
				motor(BaseRight) = -Power; }}


		void forwardEncoderwIntake(int encoderCount,int Power) {
			SensorValue[leftBaseEncoder] = 0;
			SensorValue[rightBaseEncoder] = 0;

			while (SensorValue[rightBaseEncoder] > encoderCount)
				{ motor(BaseLeft) = Power;
					motor(BaseRight) = Power;
					while (motor(BaseLeft) = Power) {
					 motor(Wheel) = -127;
					 wait1Msec(1500);
				}}

			}






/*/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  Auto Setups
*//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void Nothing() { //Nothing

}

void testingAuto() {
	SensorValue(leftBaseEncoder) = 0;
	//offset = -45;
	//armPID.isEnabled = true;
	//moveWheelTime(-100,600);
	//wait1Msec(1000);
	//moveWheelTime(100,600);
	//armPID.setpoint = armAtFloor;
	moveForward(18, -100);
	//runClawTime(100,1000);
	//wait1Msec(2000);
	//startTask(autoPID);
	writeDebugStreamLine("moving forward");

	//moveForward(12, 50, 1);
	//wait1Msec(250);
	//armPID.setpoint = armForMobile;
	wait1Msec(2000);
	//moveBackward(12, 100, 1);
	writeDebugStreamLine("finished");
	//turnToAngle(-180, 100, 2);
	//moveBackward(40, 100, 3);
	//wait1Msec(2000);
	//moveForward(35, 100, 2.5);
	wait1Msec(500);
	//armPID.setpoint = armPreload;
	//moveBackward(12, 100, 3);
}




void BlueRight() {

	 motor[intake] = -25;
	 //runArmTime(63,100);
   runChainTime(63,1000);
   runChainTime(-63,450);
	 runWheelTime(-127, 1000);
	 runForwardTime(100,1600);
	 wait10Msec(30);
	 runWheelTime(127,1100);
	 wait10Msec(10)
	 runChainTime(-63,350);
	 wait10Msec(15);
	 runClawTime(127,100);

	 runBackwardTime(127,1100);
	 runSwingLeftTime(63,1300);
	 runForwardTime(127,500);
	 runSwingLeftTime(127,450);
	 runForwardTime(127,1350);
	 runWheelTime(-127,1000);
	 runBackwardTime(127,900);
		//runForwardTime(
}


void BlueLeft() {
		motor[Intake] = -25;
	 //runArmTime(63,100);
   runChainTime(127,500);
   runChainTime(-127,375);
	 runWheelTime(-127, 1000);
	 runForwardTime(100,1800);
	 wait10Msec(30);
	 runWheelTime(127,1100);
	 wait10Msec(30);
	 runClawTime(127,100);

	 runBackwardTime(127,1700);
	 runSwingRightTime(63,700);
	 runForwardTime(127,200);
	 runSwingRightTime(127,600);
	 runForwardTime(127,1600);
	 runWheelTime(-127,1000);
	 runBackwardTime(127,1000);
	 runSwingRightTime(63,1100);
		//runForwardTime(
}

void RedRight() {

	 motor[intake] = -25;
	 //runArmTime(63,100);
   runChainTime(63,900);
   runChainTime(-63,500);
	 runWheelTime(-127, 1000);
	 runForwardTime(100,1800);
	 wait10Msec(30);
	 runWheelTime(127,1100);
	 wait10Msec(30)
	 runChainTime(-63,400);
	 runClawTime(127,100);

	 runBackwardTime(127,1700);
	 runSwingLeftTime(63,1300);
	 runForwardTime(127,100);
	 runSwingLeftTime(127,400);
	 runForwardTime(127,1500);
	 runWheelTime(-127,1000);
	 runBackwardTime(127,1000);
	 runSwingLeftTime(63,1300);
		//runForwardTime(
}

void RedLeft() {
	 motor[intake] = -25;
	 //runArmTime(63,100);
   runChainTime(127,500);
   runChainTime(-117,395);
	 runWheelTime(-127, 1000);
	 runForwardTime(100,1600);
	 wait10Msec(30);
	 runWheelTime(127,1050);
	 wait10Msec(10)
	 runClawTime(127,100);

	 runBackwardTime(127,1300);
	 runSwingRightTime(63,700);
	 runForwardTime(127,240);
	 runSwingRightTime(127,600);
	 runForwardTime(127,1850);
	 runWheelTime(-127,1000);
	 runBackwardTime(127,1000);
		//runForwardTime(
}

void Skills() {

	 motor[intake] = -25;
	 //runArmTime(63,100);
   runChainTime(63,800);
   runChainTime(-63,300);
	 runWheelTime(-127, 1000);
	 runForwardTime(100,1600);
	 wait10Msec(30);
	 runWheelTime(127,1000);
	 wait10Msec(10)
	 runChainTime(-63,350);
	 wait10Msec(15);
	 runClawTime(127,100);

	 runBackwardTime(127,1300);
	 runSwingLeftTime(63,1300);
	 runForwardTime(127,500);
	 runSwingLeftTime(127,350);
	 runForwardTime(127,1450);
	 runWheelTime(-127,1000);
	 runBackwardTime(127,750);
	 runRightTime(127,450);
	 runBackwardTime(127,1800);
	 runRightTime(127,100);
	 wait10Msec(10);
	 runWheelTime(-127,300);
	 runForwardTime(127,1100);
	 runWheelTime(127,300);
	 runBackwardTime(127,300);
	 runLeftTime(127,350);
	 runForwardTime(127,600);
	 runWheelTime(-127,300);
	 runBackwardTime(127,7000);

		//runForwardTime(
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void BlueRightfivepoint() {

	 motor[intake] = -25;
	 //runArmTime(63,100);
   runChainTime(63,1000);
   runChainTime(-63,350);
	 runWheelTime(-127, 1000);
	 runForwardTime(100,1600);
	 wait10Msec(30);
	 runWheelTime(127,1000);
	 wait10Msec(10)
	 runChainTime(-63,350);
	 wait10Msec(15);
	 runClawTime(127,100);

	 runBackwardTime(127,1000);
	 runLeftTime(127,1000);
	 runForwardTime(127,400);
	 runWheelTime(-127,1000);
	 runBackwardTime(127,400);
		//runForwardTime(
}


void BlueLeftfivepoint() {
		motor[intake] = -25;
	 //runArmTime(63,100);
   runChainTime(127,500);
   runChainTime(-127,375);
	 runWheelTime(-127, 1000);
	 runForwardTime(100,1800);
	 wait10Msec(30);
	 runWheelTime(127,1100);
	 wait10Msec(30)
	 runClawTime(127,100);

	 runBackwardTime(127,1000);
	 runRightTime(127,1000);
	 runForwardTime(127,400);
	 runWheelTime(-127,1000);
	 runBackwardTime(127,400);
		//runForwardTime(
}

void RedRightfivepoint() {

	 motor[intake] = -25;
	 //runArmTime(63,100);
   runChainTime(63,1000);
   runChainTime(-63,700);
	 runWheelTime(-127, 1000);
	 runForwardTime(100,1800);
	 wait10Msec(30);
	 runWheelTime(127,1100);
	 wait10Msec(30)
	 runClawTime(127,100);

	runBackwardTime(127,1000);
	 runLeftTime(127,1000);
	 runForwardTime(127,400);
	 runWheelTime(-127,1000);
	 runBackwardTime(127,400);
		//runForwardTime(
}

void RedLeftfivepoint() {
	 motor[intake] = -25;
	 //runArmTime(63,100);
   runChainTime(127,500);
   runChainTime(-127,375);
	 runWheelTime(-127, 1000);
	 runForwardTime(100,1600);
	 wait10Msec(30);
	 runWheelTime(127,1000);
	 wait10Msec(10)
	 runClawTime(127,100);

	 runBackwardTime(127,1000);
	 runRightTime(127,1000);
	 runForwardTime(127,250);
	 runWheelTime(-127,1000);
	 runBackwardTime(127,400);
		//runForwardTime(
}




































//
