
//bool checkFlywheel()
//{
//	if(SensorValue[flyEncoder] > flyThreshold)
//		return true;
//	else
//		return false;
//}

//task flywheelTick()
//{
//	bool lastState = false;
//	while(true)
//	{
//		if(checkFlywheel() != lastState)
//		{
//			flywheelTicks++;
//			lastState = !lastState;
//		}
//	}
//}

bool checkFlywheel()
{
	if (SensorValue[flyHall] == 0)
		return true;
	else
		return false;
}

task flywheelTick()
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







#define ballDistance 400

task pyramidControl()
{
	while(true)
	{
		if((SensorValue[ballSONAR] < ballDistance) && (SensorValue[intakepiston] == 0))
			SensorValue[intakepiston] = 1;
	}
}

//task RPMLoop()
//{
//	int rotationPiece = 1;
//	resetTimer(T1);
//	while(true)
//	{
//		if (flywheelTicks == rotationPiece*24)
//		{
//			RPM = rotationPiece*60.0/(time1[T1]/1000.0);
//			resetTimer(T1);
//			flywheelTicks = 0;
//		}
//	}
//}

task RPMLoop()
{
	const int RPMdelay = 100; // find the rpm every 100 millsec
	while(true)
	{
		//SensorValue[flyHall] = 0;
		//wait1Msec(100);
		//RPM = SensorValue[flyHall]*25;
		SensorValue[flyHall] = 0;
		wait1Msec(RPMdelay);
		RPM = (SensorValue[flyHall]*60.0*1000)/(24.0*RPMdelay); // 60 = sec to min, 1000/RPMdelay = wait time conversion

	}
}

//~~~ FLYWHEEL~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//Global variables
int error = 0;
int kpMotorValue = 0; //used for debugging
int kiMotorValue = 0;
int kdMotorValue = 0;

task flyPID()
{
	int RPM_Drop = false;
	int	errorData[2]; //averaging
	int elementNum = 2;
	for (int n = 0; n < elementNum; n++){ //initilizing
		errorData[n] = 0;
	}
	int RPMCounter = 0;
	int errorSum = 0;
	float kp = 0.38;
	float ki = 0.0085;
	float kd = 0.15;
	float k  = 0.044; //base rpm (dependant on target RPM) //0.05
	int sigmaError = 0;
	int deltaError = 0;
	int previousError = 0;

	while(true)
	{
		if (RPMCounter>elementNum-1)
		{
			RPMCounter = 0;
		}
		errorData[RPMCounter] = targetRPM - RPM;
		errorSum = 0;
		for(int k = 0; k < elementNum; k++){ //taking the sum of the past RPM data point
			errorSum += errorData[k];
		}
		error = errorSum/elementNum; //taking the average RPM

		if(abs(deltaError) > 40)
		{
			RPM_Drop = true;
			resetTimer(T2);
		}
		else if (time1[T2] > 600)
			RPM_Drop = false;

		if(motorOutput < 110 && motorOutput > 20 && abs(error) < 250 && RPM_Drop == false) //intergration restrictions
			sigmaError += error;

		deltaError = error - previousError;

		PIDOutput = error * kp + sigmaError * ki + deltaError * kd + targetRPM*k;
		//debug~~~~~~~~~~~~~~~~~~~~~~~~~
		kpMotorValue = error * kp;
		kiMotorValue = sigmaError * ki;
		kdMotorValue = deltaError * kd;
		//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
		if(PIDOutput > 127) //caping motor value
			PIDOutput = 127;
		else if(PIDOutput < 0)
			PIDOutput = 0;

		motorOutput = (TrueSpeed[PIDOutput]); //true speed + send value to motor
		previousError = error;
		wait1Msec(10);
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

task RPMGRAPH()
{
	clearTimer(T4);
	datalogClear();
	while(true)
	{
		datalogDataGroupStart();
		datalogAddValue( 0, RPM );
		datalogAddValue( 1, targetRPM );
		datalogAddValue( 2, motorOutput );
		datalogAddValue( 3, error);
		datalogAddValue( 4, SensorValue[flyEncoder] );
		datalogAddValue( 5, kpMotorValue);
		datalogAddValue( 6, kiMotorValue);
		datalogAddValue( 7, kdMotorValue);
		datalogDataGroupEnd();
		wait1Msec(10);
	}
}
