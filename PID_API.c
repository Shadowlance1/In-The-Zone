//Used to initialize the structs information for a given controller
void PIDInitializeController(PID pPID, tSensors inputSensor, float setpoint, int maxOutput, int minOutput, float mySensorScale, float integralLimit, float kP, float kI, float kD) {
	pPID.inputSensor = inputSensor;
	pPID.maxOutput = maxOutput;
	pPID.minOutput = minOutput;
	pPID.kP = kP;
	pPID.kI = kI;
	pPID.kD = kD;
	pPID.setpoint = setpoint;
	pPID.mySensorScale = mySensorScale;
	pPID.integralLimit = integralLimit;
	pPID.isEnabled = true;

}

void PIDInitializeVelocityController(PID pPID, tMotor velocityMotorInput, float setpoint, int maxOutput, int minOutput, float mySensorScale, float integralLimit, float kP, float kI, float kD) {
	pPID.velocityMotorInput = velocityMotorInput;
	pPID.maxOutput =  maxOutput;
	pPID.minOutput = minOutput;
	pPID.kP = kP;
	pPID.kI = kI;
	pPID.kD = kD;
	pPID.setpoint = setpoint;
	pPID.mySensorScale = mySensorScale;
	pPID.integralLimit = integralLimit;
	pPID.isEnabled = true;
}

bool PIDatSetpoint(PID pPID) {
	if (abs(pPID.error) <= pPID.tolerance) {
		return true;
	}
	else {
		return false;
	}
}

void setpoint(PID pPID, int setpoint) {
	pPID.setpoint = setpoint;
}
//only use in task!!!
float PIDControl(PID pPID) {

	pPID.error = pPID.setpoint - ((SensorValue[pPID.inputSensor] * pPID.mySensorScale));

	// integral - if Ki is not 0
	if( pPID.kI != 0 )
	{
		// If we are inside controlable window then integrate the error
		if( abs(pPID.error) < pPID.integralLimit) {
			pPID.integral += pPID.error;
		}
		else{
			pPID.integral = 0;
		}
	}
	else {
		pPID.integral = 0;
	}

	pPID.derivative = pPID.error - pPID.previousError;
	pPID.previousError = pPID.error;

	float PID_motorOutput = (pPID.kP * pPID.error) + (pPID.kI * pPID.integral * .02) + (pPID.kD * (pPID.derivative/.02));
	pPID.output = PID_motorOutput;

	if (PID_motorOutput > pPID.maxOutput) {
		return pPID.maxOutput;
	}
	else if (PID_motorOutput < -pPID.maxOutput) {
		return -pPID.maxOutput;
	}
	else if (PID_motorOutput > 0 && PID_motorOutput < pPID.minOutput) {
		return pPID.minOutput;
	}
	else if (PID_motorOutput < 0 && PID_motorOutput > -pPID.minOutput) {
		return -pPID.minOutput;
	}
	else {
		return PID_motorOutput;
	}

}
//only use in task!!!
float PIDVelocityControl(PID pPID) {

	pPID.error = pPID.setpoint - (getMotorVelocity(pPID.velocityMotorInput));

	// integral - if Ki is not 0
	if( pPID.kI != 0 )
	{
		// If we are inside controlable window then integrate the error
		if( abs(pPID.error) < pPID.integralLimit) {
			pPID.integral += pPID.error;
		}
		else{
			pPID.integral = 0;
		}
	}
	else {
		pPID.integral = 0;
	}

	pPID.derivative = pPID.error - pPID.previousError;
	pPID.previousError = pPID.error;

	float PID_motorOutput = (pPID.kP * pPID.error) + (pPID.kI * pPID.integral * .05) + (pPID.kD * (pPID.derivative / .05));

	if (PID_motorOutput > 127) {
		return 127;
	}
	else if (PID_motorOutput < -127) {
		return -127;
	}
	else {
		return PID_motorOutput;
	}


}
