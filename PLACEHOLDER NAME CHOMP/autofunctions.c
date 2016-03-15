// ** DRIVE FUNCTION **
// utilizes a feed-forward PID loop to accurately move/rotate to target
// based on the premise that absolute encoder values on both sides should be equal at all times
// runs PID loop when both encoder values are equal & P loop to compensate when they are not equal
void driveAD (char* direction, int targetTicks)
{
	// initialize constants & variables
	float kp = 0.01; 	// NEEDS TUNING
	float ki = 0;
	float kd = 0.2;
	int EncoderL = 0;
	int EncoderR = 0;
	int leftPower = 0;
	int rightPower = 0;
	int error = 0;
	int previousError = targetTicks;
	int sigmaError = 0;
	int deltaError = 0;

	// drive while target has not been reached
	while ((abs(SensorValue[leftEncoder]) + abs(SensorValue[rightEncoder])) / 2 < targetTicks)
	{
		// set absolute encoder values
		EncoderL = abs(SensorValue[leftEncoder]);
		EncoderR = abs(SensorValue[rightEncoder]);

		// when both sides are driving at same speed
		if (EncoderL == EncoderR)
		{
			error = targetTicks - (EncoderL + EncoderR) / 2;	// get error
			sigmaError += error;															// integrate error
			deltaError = error - previousError;								// differentiate error

			// get feed-forward PID output
			leftPower += error * kp + sigmaError * ki + deltaError * kd;
			rightPower += error * kp + sigmaError * ki + deltaError * kd;
		}

		// when right side is faster than left side
		else if (EncoderL < EncoderR)
		{
			error = targetTicks - EncoderL; // get error
			rightPower -= error * kp; 			// subtract P-loop output from right side
		}

		// when left side is faster than right side
		else if (EncoderL > EncoderR)
		{
			error = targetTicks - EncoderR;	// get error
			leftPower -= error * kp;				// subtract P-loop output from left side
		}

		// clamp motor power in between 0 & 127
		leftPower = leftPower > 127 ? 127 : leftPower;
		leftPower = leftPower < 0 ? 0 : leftPower;
		rightPower = rightPower > 127 ? 127 : rightPower;
		rightPower = rightPower < 0 ? 0 : rightPower;

		// set motor powers depending on direction parameter
		if (direction == "forward")
		{
			leftOutput = leftPower;
			rightOutput = rightPower;
		}
		else if (direction == "backward")
		{
			leftOutput = -leftPower;
			rightOutput = -rightPower;
		}
		if (direction == "left")
		{
			leftOutput = -leftPower;
			rightOutput = rightPower;
		}
		else if (direction == "right")
		{
			leftOutput = leftPower;
			rightOutput = -rightPower;
		}

		// set previous error
		previousError = error;
	}
	leftOutput = 0;
	rightOutput = 0;
}
