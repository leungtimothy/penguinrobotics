// RPM Task
task rpmCalculation()
{
	while(true)
	{
		SensorValue[fwEncoder] = 0;
		wait1Msec(rpmDelay);

		// conversion: 1 revolution per 120 ticks, 1000 milliseconds per 1 seconds, 60 seconds per 1 minute
		RPM = abs(SensorValue[fwEncoder]) * 500 / rpmDelay;

		isNewRPM = true;
	}
}

// PID Flywheel Velocity Control, P term for beginning, I term for middle, D for end
task pidControl()
{
	// initialize constants & variables
	const float kp = 0.000075;
	const float ki = 0.00075;
	const float kd = 0.03;
	float PIDOutput = 0;
	int error = 0;
	int previousError;
	int sigmaError = 0;
	int deltaError = 0;

	datalogClear();
	while (true)
	{
		if (isNewRPM)
		{
			if (targetRPM != 0)
			{
				error = targetRPM - RPM; 						// find error
				deltaError = error - previousError;	// differentiate error
				// integrate errors when error is not changing fast enough
				if (abs(deltaError) < deltaThreshold)
					sigmaError += error;
				else
					sigmaError = 0;

				PIDOutput = PIDOutput > 127 ? 127 :	PIDOutput < 0 ? 0 : 																							// clamp output between 0 and 127
												error < errorThreshold ? PIDOutput + error * kp + sigmaError * ki + deltaError * kd :	// PID when output is in control zone
																								 PIDOutput + error * kp + sigmaError * ki;										// PI to accelerate into control zone

				fwPower = PIDOutput;								// send to motors

				isNewRPM = false;										// reset boolean to use new RPMs only

				previousError = error;							// set new previous error

				datalogDataGroupStart();
				datalogAddValue(0, fwPower);
				datalogAddValue(1, RPM);
				datalogAddValue(2, targetRPM);
				datalogDataGroupEnd();
			}
			else
				fwPower = 0;

			// for debugging
			if (RPM != 0)
				writeDebugStream("Output: %i\t\t\tP: %i\t\t\tI: %i\t\t\tD: %i\t\t\tRPM: %i\n", fwPower, error, sigmaError, deltaError, RPM);
		}
	}
}

// Launcher Motor Controller
task motorControl()
{
	while (true)
		motor[FlyL] = motor[FlyR1] = motor[FlyR2] = motor[FlyR3] = fwPower;	// set flywheel speed
		motor[DriveL] = leftOutput;
		motor[DriveR1] = motor[DriveR2] = rightOutput;
}

// Ball Inhibitor
task ballInhibitor()
{
	while (true)
	{
		// check if ball is in place for shot
		isBallReady = (SensorValue[ballSensorTop] < ballThreshold);

		// check if RPM is suitable
		isRPMReady = (targetRPM > RPM && targetRPM != 0);
	}
}

// Ball Counter
task ballCounter()
{
	while (true){
		if (SensorValue[ballSensorTop] < ballThreshold){
			ballCount++;
			while (SensorValue[ballSensorTop] < ballThreshold){};
		}
	}
}
