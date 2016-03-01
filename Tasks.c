// Function to return RPM value after 100ms poll time
int getRPM()
{
	SensorValue[flyHall] = 0;
	wait1Msec(rpmDelay);

	// conversion: 1 revolution per 24 ticks, 1000 milliseconds per 1 seconds, 60 seconds per 1 minute
	return (SensorValue[flyHall] / rpmDelay) * (1 / 24) * (1000 / 1) * (60 / 1);
}

// PID Flywheel Velocity Control
task pidControl()
{
	// initialize constants & variables
	float kp = 0.0002;
	float kd = 0.15;
	float PIDOutput = 0;
	int error = 0;
	int deltaError = 0;
	int previousError = 3000; // prevent output capping during first loop

	while (true)
	{
		RPM = getRPM();															// find instanteous RPM
		error = targetRPM - RPM; 										// find error
		deltaError = error - previousError; 				// differentiate error
		PIDOutput += error * kp + deltaError * kd;	// calculate PID output

		// clamp PID output between 0 & 127
		if (PIDOutput > 127)
			PIDOutput = 127;
		else if (PIDOutput < 0)
			PIDOutput = 0;

		previousError = error;	// set previous error

		// turn motors off if target RPM is 0 & reset previous error to prevent output capping
		if (targetRPM == 0)
		{
			PIDOutput = 0;
			previousError = 3000;
		}

		fwOutput = PIDOutput; // send motor output


		//if (PIDOutput != 0)
		//	writeDebugStream("Output: %i\t\tP: %i\t\tD: %i\t\tRPM: %i\n", PIDOutput, error, deltaError);
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
