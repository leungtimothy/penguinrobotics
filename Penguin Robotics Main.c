#pragma config(Sensor, in1,    flyEncoder,     sensorLineFollower)
#pragma config(Sensor, in2,    ballSensorBottom, sensorLineFollower)
#pragma config(Sensor, in3,    ballSensorTop,  sensorLineFollower)
#pragma config(Sensor, dgtl1,  flywheelSwitch, sensorTouch)
#pragma config(Sensor, dgtl12, intakepiston,   sensorDigitalOut)
#pragma config(Sensor, I2C_1,  fly1,           sensorNone)
#pragma config(Motor,  port1,           DriveR1,       tmotorVex393_HBridge, openLoop)
#pragma config(Motor,  port2,           FlyR1,         tmotorVex393_MC29, openLoop, reversed)
#pragma config(Motor,  port3,           FlyR2,         tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port4,           FlyL,          tmotorVex393_MC29, openLoop, reversed)
#pragma config(Motor,  port6,           FlyTop,        tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port7,           DriveL,        tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port8,           Intake2,       tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port9,           Intake1,       tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port10,          DriveR2,       tmotorVex393_HBridge, openLoop)
#pragma config(DatalogSeries, 0, "Timer", Timers, time100, T1, 1000)
#pragma config(DatalogSeries, 1, "", Properties, averageBatteryLevel, , 1000)
#pragma config(DatalogSeries, 2, "", Properties, immediateBatteryLevel, , 500)
//*!!Code automatically generated by 'ROBOTC' configuration wizard               !!*//

#pragma platform(VEX)

//Competition Control and Duration Settings
#pragma competitionControl(Competition)
#pragma autonomousDuration(20)
#pragma userControlDuration(120)

#include "Vex_Competition_Includes.c"   //Main competition background code...do not modify!

#define flyThreshold 1000
#define ballThreshold 2500
#define integralThreshold 0

int flywheelSpeed = 0;
int motorOutput = 0;
float RPM = 0;
float currTick = 0;
float targetRPM = 0;
float offset = 0;
float battery = 0;
float flywheelTicks = 0;
int FlyTop1 = 0;

//int flyWheelFilter[5];






const unsigned int TrueSpeed[128] =
{
	0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
	0, 21, 21, 21, 22, 22, 22, 23, 24, 24,
	25, 25, 25, 25, 26, 27, 27, 28, 28, 28,
	28, 29, 30, 30, 30, 31, 31, 32, 32, 32,
	33, 33, 34, 34, 35, 35, 35, 36, 36, 37,
	37, 37, 37, 38, 38, 39, 39, 39, 40, 40,
	41, 41, 42, 42, 43, 44, 44, 45, 45, 46,
	46, 47, 47, 48, 48, 49, 50, 50, 51, 52,
	52, 53, 54, 55, 56, 57, 57, 58, 59, 60,
	61, 62, 63, 64, 65, 66, 67, 67, 68, 70,
	71, 72, 72, 73, 74, 76, 77, 78, 79, 79,
	80, 81, 83, 84, 84, 86, 86, 87, 87, 88,
	88, 89, 89, 90, 90,127,127,127
};

/////////////////////////////////////////////////////////////////////////////////////////
//
//                          Pre-Autonomous Functions
//
// You may want to perform some actions before the competition starts. Do them in the
// following function.
//
/////////////////////////////////////////////////////////////////////////////////////////

void pre_auton()
{
	// Set bStopTasksBetweenModes to false if you want to keep user created tasks running between
	// Autonomous and Tele-Op modes. You will need to manage all user created tasks if set to false.
	bStopTasksBetweenModes = true;

	// All activities that occur before the competition starts
	// Example: clearing encoders, setting servo positions, ...
}

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


task RPMLoop()
{
	resetTimer(T1);
	float oldTime = getTimer(T1,milliseconds);

	//for(int n = 0; n < 5; n++){ //TAKING THE FLYWHEEL RPM AVERAGE
	//	flyWheelFilter[n] = 0;
	//}

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
	float kp = 0.025;
	float ki = 0;
	float kd = 0.00;
	int error = 0;
	int sigmaError = 0;
	int deltaError = 0;
	int previousError = 0;

	while(true)
	{
		if(targetRPM > 0)
			error = targetRPM - RPM;
		else
			error = 0;
		if(error > integralThreshold)
			sigmaError += error;
		else
			sigmaError = 0;
		deltaError = error - previousError;
		motorOutput += error * kp + sigmaError * ki + deltaError * kd;
		if(motorOutput > 127)
			motorOutput = 127;
		if(motorOutput < 0)
			motorOutput = 0;
		previousError = error;
		writeDebugStream("%i\n", motorOutput);
	}
}

task pidControl()
{
	while(true)
	{
		motor[FlyL] = motor[FlyR1] = motor[FlyR2] = motorOutput;
		motor[FlyTop] =FlyTop1;
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

/////////////////////////////////////////////////////////////////////////////////////////
//
//                                 Autonomous Task
//
// This task is used to control your robot during the autonomous phase of a VEX Competition.
// You must modify the code to add your own robot specific commands here.
//
/////////////////////////////////////////////////////////////////////////////////////////

task autonomous()
{
	// .....................................................................................
	// Insert user code here.
	// .....................................................................................

	AutonomousCodePlaceholderForTesting();  // Remove this function call once you have "real" code.
}

/////////////////////////////////////////////////////////////////////////////////////////
//
//                                 User Control Task
//
// This task is used to control your robot during the user control phase of a VEX Competition.
// You must modify the code to add your own robot specific commands here.
//
/////////////////////////////////////////////////////////////////////////////////////////
bool btn7UPressed = false;
bool btn7DPressed = false;

task usercontrol()
{
	// User control code here, inside the loop
	//startTask(motorControl);
	startTask(flyPID);
	startTask(pidControl);
	startTask(RPMLoop);
	startTask(flywheelTick);

	while (true)
	{
		battery = nAvgBatteryLevel;
		motor[DriveL] = vexRT[Ch3];
		motor[DriveR1] = motor[DriveR2] = vexRT[Ch2];
		if(vexRT[Btn8D] == 1)
		{
			targetRPM = 0;
			FlyTop1 = 0;
		}
		else if(vexRT[Btn8R] == 1)
		{
			targetRPM = 1600;
			FlyTop1 = 127;
		}
		else if(vexRT[Btn8L] == 1)
		{
			targetRPM = 2000;
			FlyTop1 = 127;
		}
		else if(vexRT[Btn8U] ==1)
		{
			targetRPM = 2500;
			FlyTop1 = 127;
		}

		if(vexRT[Btn7U] == 1 && btn7UPressed == false)
		{
			targetRPM += 200;
			FlyTop1 = 127;
			if (targetRPM > 2500)
				targetRPM = 2500;
			btn7UPressed = true;
		}
		else if(vexRT[Btn7U] == 0) //&& btn7UPressed == true)
			btn7UPressed = false;

		if(vexRT[Btn7D] == 1 && btn7DPressed == false)
		{
			targetRPM -= 200;
			if (targetRPM < 0){
				targetRPM = 0;
				FlyTop1 = 0;
			}
			btn7DPressed = true;
		}
			else if(vexRT[Btn7D] == 0) //&& btn7DPressed == true)
			btn7DPressed = false;


		//		bool checkFlywheel()
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




		if(vexRT[Btn7DXmtr2] ==1)
		{
			SensorValue[intakepiston] = 1;
		}
		else if(vexRT[Btn7UXmtr2] ==1)
		{
			SensorValue[intakepiston] = 0;
		}

		if(vexRT[Btn5DXmtr2] == 1)
		{
			motor[Intake1]=-127;
		}
		else if (vexRT[Btn5UXmtr2] ==1)
		{
			motor[Intake1]=127;
		}
		else
		{
			motor[Intake1]=0;
		}

		if(vexRT[Btn6Dxmtr2] == 1)
		{
			motor[Intake2]=-127;
		}
		else if(vexRT[Btn6UXmtr2] ==1)
		{
			motor[Intake2]=127;
		}
		else
		{
			motor[Intake2]=0;
		}
		
		// meme comment XDD
		// meme 2
	}
}
