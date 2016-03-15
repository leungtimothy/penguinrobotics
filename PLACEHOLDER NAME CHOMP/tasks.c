// RPM Task
task rpmCalc()
{
	while(true)
	{
		SensorValue[flyHall] = 0;
		wait1Msec(rpmDelay);

		// conversion: 1 revolution per 24 ticks, 1000 milliseconds per 1 seconds, 60 seconds per 1 minute
		RPM = (SensorValue[flyHall] / rpmDelay) * (1 / 24) * (1000 / 1) * (60 / 1);

		isNewRPM = true;
	}
}

// PD Flywheel Velocity Control
task pdControl()
{
	// initialize constants & variables
	float kp = 0.0002;
	float kd = 0.15;
	float PDOutput = 0;
	int error = 0;
	int deltaError = 0;
	int previousError = 3000; // prevent output capping during first loop

	while (true)
	{
		if (isNewRPM)
		{
			error = targetRPM - RPM; 										// find error
			deltaError = error - previousError; 				// differentiate error
			previousError = error;											// set previous error
			PDOutput += error * kp + deltaError * kd;	// calculate PD output

			// clamp PID output between 0 & 127
			PDOutput = PDOutput > 127 ? 127 : PDOutput < 0 ? 0 : PDOutput;

			// turn motors off if target RPM is 0 & reset previous error to prevent output capping
			if (targetRPM == 0)
			{
				PDOutput = 0;
				previousError = 3000;
			}

			fwOutput = PDOutput; // send motor output

			isNewRPM = false;

			//if (PDOutput != 0)
			//	writeDebugStream("Output: %i\t\tP: %i\t\tD: %i\t\tRPM: %i\n", PDOutput, error, deltaError);
		}
	}
}

// PI Flywheel Velocity Control, P term for ends, I term for explosive middle
task piControl()
{
	// initialize constants & variables
	float kp = 0.0002;
	float ki = 0.005;
	float PIOutput = 0;
	int error = 0;
	int sigmaError = 0;

	while (true)
	{
		if (isNewRPM)
		{
			error = targetRPM - RPM; 										// find error

			// integrate significant errors
			if (error > integralThreshold)
				sigmaError += error;
			else
				sigmaError = 0;

			PIOutput += error * kp + sigmaError * ki;		// calculate PI output

			// clamp PID output between 0 & 127, send to motors
			fwOutput = PIOutput > 127 ? 127 : (PIOutput < 0 || targetRPM == 0) ? 0 : PIOutput;

			// reset boolean to use new RPMs only
			isNewRPM = false;

			//if (PIDOutput != 0)
			//	writeDebugStream("Output: %i\t\tP: %i\t\tD: %i\t\tRPM: %i\n", PIDOutput, error, deltaError);
		}
	}
}

// Launcher Motor Controller
task motorControl()
{
	while (true)
		motor[FlyL] = motor[FlyR1] = motor[FlyR2] = fwOutput;	// set flywheel speed
		motor[DriveL] = leftOutput;
		motor[DriveR1] = motor[DriveR2] = rightOutput;
}

// Ball Inhibitor
task ballInhibitor()
{
	while (true)
	{
		// check if ball is in place for shot
		isBallReady = SensorValue[ballSensorTop] < ballThreshold;

		// check if RPM is suitable
		isRPMReady = (targetRPM > (RPM - 50) && targetRPM != 0);
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
