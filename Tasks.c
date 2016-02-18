// RPM Calculation System
bool checkFlywheel()
{
	if (SensorValue[flyHall] == 0)
		return true;
	else
		return false;
}

task fwTickCount()
{
	bool lastState = false;
	while (true)
	{
		if (checkFlywheel() != lastState)
		{
			flywheelTicks++;
			lastState = !lastState;
		}
	}
}

task rpmCalc()
{
	const int countNumOfSpin = 1; // wait until x rotation
	const int ticksPerRotation = 24; //24 ticks/rev
	while(true)
	{
		flywheelTicks = 0;
		wait1Msec(1000);
		RPM = flywheelTicks*2.4;
	}
}

// TBH Flywheel Velocity Control
bool signbit(int input)
{
	bool output = input < 0 ? true : false;
	return output;
}

task tbhControl()
{
	// initialize constants & variables
	float kGain = 0.00005;
	float TBHOutput = 1;
	float kZero = 0;
	int error = 0;
	int previousError = 0;

	while (true)
	{
		error = targetRPM - RPM;		// find error
		TBHOutput += kGain * error;	// integrate error

		// clamp TBH variable between 0 & 1
		if (TBHOutput > 1)
			TBHOutput = 1;
		else if (TBHOutput < 0)
			TBHOutput = 0;

		// check for zero error crossing
		if (signbit(error) != signbit(previousError))
		{
			TBHOutput = 0.5 * (TBHOutput + kZero);	// take back half
			kZero = TBHOutput;											// reset zero
		}
		motorOutput = TBHOutput > 1 ? 127 : ((127 * TBHOutput) + 0.5); // send motor output
		previousError = error; // set previous error
	}
}

// PID Flywheel Velocity Control
task pidControl()
{
	// initialize constants & variables
	float kp = 0.0000096;
	float ki = 0.000016;
	float kd = 0.0000004;
	float PIDOutput = 0;
	int error = 0;
	int sigmaError = 0;
	int deltaError = 0;
	int previousError = 0;

	while (true)
	{
		error = targetRPM - RPM; // find error

		// integrate error
		if (error > integralThreshold)
			sigmaError += error;
		else
			sigmaError = 0;
		if (sigmaError > 50) // limit integration to 50
			sigmaError = 50;
		deltaError = error - previousError; 													// differentiate error
		PIDOutput += error * kp + sigmaError * ki + deltaError * kd;	// calculate PID output

		// clamp PID output between 0 & 127
		if (PIDOutput > 127)
			PIDOutput = 127;
		else if (PIDOutput < 0)
			PIDOutput = 0;
		motorOutput = PIDOutput; 	// send motor output
		previousError = error;		// set previous error

		/* USED FOR DEBUGGING
		if (PIDOutput != 0)
			writeDebugStream("Output: %i\t\tP: %i\t\tI: %i\t\tD: %i\n", PIDOutput, error, sigmaError, deltaError);
		*/
	}
}

// Launcher Motor Controller
task motorControl()
{
	while (true)
	{
		motor[FlyL] = motor[FlyR1] = motor[FlyR2] = motorOutput;	// set bottom wheel speed
		motor[FlyTop] = topWheel;																	// set top wheel speed
	}
}

// Ball Inhibitor
task ballInhibitor()
{
	while (true)
	{
		// check if ball is in place for shot
		if (SensorValue[ballSensorTop] < ballThreshold)
			ballReady = true;
		else
			ballReady = false;
		// check if RPM is suitable
		if ((targetRPM > (RPM - 50)) && (targetRPM < (RPM + 50)) && (targetRPM != 0))
			RPMReady = true;
		else
			RPMReady = false;
	}
}

// Ball Counter
task ballCounter()
{
	while (true){
		if (SensorValue[ballSensorTop] < ballThreshold){
			ballCount++;
			while (SensorValue[ballSensorTop] < nullThreshold){};
		}
	}
}
