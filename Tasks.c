
bool checkFlywheel()
{
	if(SensorValue[flyEncoder] > flyThreshold)
		return true;
	else
		return false;
}

task flywheelTick()
{
	bool lastState = false;
	while(true)
	{
		if(checkFlywheel() != lastState)
		{
			flywheelTicks++;
			lastState = !lastState;
		}
	}
}

//task RPMLoop()
//{
//	while(true)
//	{

//		wait1Msec(25);
//		currTick = SensorValue[enc1];
//		RPM = currTick*(2400/360);
//	//	writeDebugStream("currTick is: %f\n", currTick);
//		writeDebugStream("RPM is: %f\n", RPM);
//		offset = (RPM - targetRPM)*0.5;
//		if((flywheelSpeed - offset) > 127)
//		{
//			flywheelSpeed = 127;
//		}
//		else if((flywheelSpeed - offset) < -127)
//		{
//			flywheelSpeed = -127;
//		}
//		else
//		{
//			flywheelSpeed = flywheelSpeed - offset;
//		}
//		motor[FL1]=motor[FL2]=motor[FR1]=motor[FR2]=TrueSpeed[flywheelSpeed];
//		currTick = 0;
//		SensorValue[enc1] = 0;
//	}
//}

task RPMLoop()
{
	resetTimer(T1);
	float oldTime = getTimer(T1,milliseconds);
	while(true)
	{
		if(flywheelTicks == 5)
		{
			float timeElapsed = (getTimer(T1,milliseconds) - oldTime);
			//writeDebugStream("timeElapsed = %f\n", timeElapsed);
			RPM = 60000/timeElapsed;
			//writeDebugStream("%f, %i\n", RPM, targetRPM);
			//datalogDataGroupStart();
			//datalogAddValue( 0, RPM );
			//datalogDataGroupEnd();
			flywheelTicks = 0;
			resetTimer(T1);
			oldTime = getTimer(T1,milliseconds);
		}
		else if	((getTimer(T1,milliseconds) - oldTime) > 2000)
		{
			RPM = 0;
			PIDOutput = 0;
		}
	}
}


task RPMLoop2()
{
	while(true)
	{
		// Sample period
		wait1Msec(25);
		// Retrieve ticks over sample period
		//currTick = SensorValue[enc1];

		// Convert encoder ticks to RPM
		RPM = -currTick*(2400/360);


		// Debug
		//	writeDebugStream("currTick is: %f\n", currTick);
		datalogDataGroupStart();
		datalogAddValue( 0, RPM );
		datalogAddValue( 1, nAvgBatteryLevel);
		datalogDataGroupEnd();
		//writeDebugStream("RPM is: %f\n", RPM);

		// Reset encoder
		currTick = 0;
		//SensorValue[enc1] = 0;
	}
}

task flyPID()
{
	float kp = 0.35;
	float ki = 0.007;
	float kd = 0.08;
	int error = 0;
	int sigmaError = 0;
	int deltaError = 0;
	int previousError = 0;
	while(true)
	{
		error = targetRPM - RPM;
		if(error > integralThreshold)
			sigmaError += error;
		else
			sigmaError = 0;
		if(sigmaError > 50)
			sigmaError = 50;
		deltaError = error - previousError;
		PIDOutput += error * kp + sigmaError * ki + deltaError * kd;
		if(PIDOutput > 127)
			PIDOutput = 127;
		else if(PIDOutput < 0)
			PIDOutput = 0;
		motorOutput = PIDOutput;
		previousError = error;
		if (PIDOutput != 0)
			writeDebugStream("Output: %i\t\tP: %i\t\tI: %i\t\tD: %i\n", PIDOutput, error, sigmaError, deltaError);
	}
}

bool signbit(int input)
{
	if(input < 0)
		return true;
	else
		return false;
}

task tbhControl()
{
	float kGain = 0.01;
	float TBHOutput = 0;
	float kTBH = 0;
	int error = 0;
	int previousError = 0;
	while(true)
	{
		error = targetRPM - RPM;
		TBHOutput += kGain * error;
		if(TBHOutput > 1)
			TBHOutput = 1;
		else if(TBHOutput < 0)
			TBHOutput = 0;
		if(signbit(error) != signbit(previousError))
		{
			TBHOutput = kTBH = (TBHOutput + kTBH) / 2;
			previousError = error;
		}
		motorOutput = TBHOutput;
	}
}

task pidControl()
{
	while(true)
	{
		motor[FlyL] = motor[FlyR1] = motor[FlyR2] = motorOutput;
		motor[FlyTop] = FlyTop1;
	}
}
