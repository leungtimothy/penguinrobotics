// RPM Calculation System
task rpmCalc()
{
	SensorValue[flyHall] = 0;
	resetTimer(T1);
	float oldTime = getTimer(T1,milliseconds);
	while(true)
	{
		if(SensorValue[flyHall] >= 24)
		{
			float timeElapsed = (getTimer(T1,milliseconds) - oldTime);
			RPM = 60000/timeElapsed;
			SensorValue[flyHall] = 0;
			resetTimer(T1);
			oldTime = getTimer(T1,milliseconds);
		}
	}
	 //while(true)
	 //{
		// SensorValue[flyHall] = 0;
		// wait1Msec(100);
		// RPM = SensorValue[flyHall] * 25;
	 //}
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
		fwOutput = TBHOutput > 1 ? 127 : ((127 * TBHOutput) + 0.5); // send motor output
		previousError = error; // set previous error
	}
}

// PID Flywheel Velocity Control
task pidControl()
{
	// initialize constants & variables
	float kp = 0.0000625;
	float kd = 0.1;
	float PIDOutput = 0;
	int error = 0;
	int deltaError = 0;
	int previousError = 3000; // avoid first time cap

	while (true)
	{
		error = targetRPM - RPM; 										// find error
		deltaError = error - previousError; 				// differentiate error
		PIDOutput += error * kp + deltaError * kd;	// calculate PID output

		// clamp PID output between 0 & 127
		if (PIDOutput > 127)
			PIDOutput = 127;
		else if (PIDOutput < 0)
			PIDOutput = 0;

		// turn motors off if target RPM is 0
		if (targetRPM == 0)
		{
			PIDOutput = 0;
			previousError = 3000;
		}

		fwOutput = PIDOutput; 	// send motor output
		previousError = error;		// set previous error

		if (PIDOutput != 0)
			writeDebugStream("Output: %i\t\tP: %i\t\tD: %i\n", PIDOutput, error, deltaError);
	}
}

// Launcher Motor Controller
task fwControl()
{
	while (true)
	{
		motor[FlyL] = motor[FlyR1] = motor[FlyR2] = fwOutput;	// set bottom wheel speed
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
