//// RPM Task
//task rpmCalc()
//{
//	while(true)
//	{
//		SensorValue[flyHall] = 0;
//		wait1Msec(rpmDelay);

//		// conversion: 1 revolution per 24 ticks, 1000 milliseconds per 1 seconds, 60 seconds per 1 minute
//		RPM = (SensorValue[flyHall] / rpmDelay) * (1 / 24) * (1000 / 1) * (60 / 1);

//		newRPM = true;
//	}
//}

// PID Flywheel Velocity Control
task pidControl()
{
	// initialize constants & variables
	float kp = 0.0002; // TO BE CHANGED
	float kd = 0.15; // TO BE CHANGED
	float PIDOutput = 0;
	int error = 0;
	int deltaError = 0;
	int previousError = 3000; // prevent output capping during first loop

	while (true)
	{
		if (newRPM)
		{
			error = targetRPM - RPM; 										// find error
			deltaError = error - previousError; 				// differentiate error
			previousError = error;											// set previous error
			PIDOutput += error * kp + deltaError * kd;	// calculate PID output

			// clamp PID output between 0 & 127
			if (PIDOutput > 127)
				PIDOutput = 127;
			else if (PIDOutput < 0)
				PIDOutput = 0;

			// turn motors off if target RPM is 0 & reset previous error to prevent output capping
			if (targetRPM == 0)
			{
				PIDOutput = 0;
				previousError = 3000;
			}

			fwOutput = PIDOutput; // send motor output

			newRPM = false;

			//if (PIDOutput != 0)
			//	writeDebugStream("Output: %i\t\tP: %i\t\tD: %i\t\tRPM: %i\n", PIDOutput, error, deltaError);
		}
	}
}

// Launcher Motor Controller
task fwControl()
{
	while (true)
		motor[TowerM1] = motor[TowerM2] = motor[TowerM3] = fwOutput;	// set flywheel speed
}
