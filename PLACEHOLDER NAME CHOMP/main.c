#pragma config(Sensor, in1,    ballSensorTop,  sensorLineFollower)
#pragma config(Sensor, in2,    ballSensorBottom, sensorLineFollower)
#pragma config(Sensor, in7,    gyro,           sensorGyro)
#pragma config(Sensor, in8,    LineFollower,   sensorLineFollower)
#pragma config(Sensor, dgtl1,  intakePiston,   sensorDigitalOut)
#pragma config(Sensor, dgtl2,  flyHall,        sensorRotation)
#pragma config(Sensor, dgtl5,  fwEncoder,      sensorQuadEncoder)
#pragma config(Sensor, dgtl7,  leftEncoder,    sensorQuadEncoder)
#pragma config(Sensor, dgtl9,  rightEncoder,   sensorQuadEncoder)
#pragma config(Sensor, dgtl11, ballSONAR,      sensorSONAR_raw)
#pragma config(Motor,  port1,           DriveR1,       tmotorVex393_HBridge, openLoop, reversed)
#pragma config(Motor,  port2,           FlyR1,         tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port3,           FlyR2,         tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port4,           Elevator,      tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port5,           Intake,        tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port7,           FlyL,          tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port8,           DriveL,        tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port9,           FlyR3,         tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port10,          DriveR2,       tmotorVex393_HBridge, openLoop, reversed)
//*!!Code automatically generated by 'ROBOTC' configuration wizard               !!*//

#pragma platform(VEX)

//Competition Control and Duration Settings
#pragma competitionControl(Competition)
#pragma autonomousDuration(20)
#pragma userControlDuration(120)

#define flyThreshold 1000
#define ballThreshold 2500
#define SONARThreshold 350
#define errorThreshold 500
#define deltaThreshold 75
#define rpmDelay 20

int targetRPM = 0;
int RPM = 0;
int fwPower = 0;
int leftOutput = 0;
int rightOutput = 0;
int ballCount = 0;

bool isBallReady = false;
bool isRPMReady = false;
bool isNewRPM = false;

#include "Vex_Competition_Includes.c"   //Main competition background code...do not modify!
#include "tasks.c"
#include "autofunctions.c"

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
	startTask(rpmCalculation);
	startTask(pidControl);
	startTask(motorControl);
	startTask(ballInhibitor);
	startTask(ballCounter);
  targetRPM = 2000;
	while(!isRPMReady){};
	wait1Msec(2000);
	targetRPM = 2400;
	while(!isRPMReady){};
	wait1Msec(2000);
	targetRPM = 2700;
	while(!isRPMReady){};
	wait1Msec(1000);
	shootBall(2);
}

task usercontrol()
{
	//Task Startup
	startTask(rpmCalculation);
	startTask(pidControl);
	startTask(motorControl);
	startTask(ballInhibitor);

	// Booleans for toggle check
	bool isBtn7LPressed = false;
	bool isBtn7RPressed = false;
	while (true)
	{
		// Tank Drive w/ deadzone of 20
	  if (vexRT[Ch3] > 20)
	  	motor[DriveL] = TrueSpeed[abs(vexRT[Ch3])];
		else if (vexRT[Ch3] < -20)
			motor[DriveL] = -TrueSpeed[abs(vexRT[Ch3])];
		else
			motor[DriveL] = 0;
		if (vexRT[Ch2] > 20)
			motor[DriveR1] = motor[DriveR2] = TrueSpeed[abs(vexRT[Ch2])];
		else if (vexRT[Ch2] < -20)
			motor[DriveR1] = motor[DriveR2] = -TrueSpeed[abs(vexRT[Ch2])];
		else
			motor[DriveR1] = motor[DriveR2] = 0;

		// RPM Increment
		if (vexRT[Btn7L] && isBtn7LPressed == false)
		{
			targetRPM = targetRPM > 0 ? targetRPM - 100 : 0;
			isBtn7LPressed = true;
		}
		else if (vexRT[Btn7L] == 0)
			isBtn7LPressed = false;
		if (vexRT[Btn7R] && isBtn7RPressed == false)
		{
			targetRPM = targetRPM < 2700 ? targetRPM + 100 : 2700;
			isBtn7RPressed = true;
		}
		else if (vexRT[Btn7R] == 0)
			isBtn7RPressed = false;

		// Intake Piston Control
		if (vexRT[Btn7U])
			SensorValue[intakePiston] = 0;
		else if (vexRT[Btn7D] || SensorValue[ballSONAR] < SONARThreshold)
			SensorValue[intakePiston] = 1;

		// Front Intake Control
		if (vexRT[Btn6U])
			motor[Intake] = 127;
		else if (vexRT[Btn6D])
			motor[Intake] = -127;
		else
			motor[Intake] = 0;

		// Elevator Control
		if (vexRT[Btn5U] && (isBallReady == false || (isBallReady && isRPMReady)))
			motor[Elevator] = 127;
		else if (vexRT[Btn5D])
			motor[Elevator] = -127;
		else
			motor[Elevator] = 0;

		// Flywheel Control
		if (vexRT[Btn8D])
			targetRPM = 0;
		else if (vexRT[Btn8R])
			targetRPM = 2200;
		else if (vexRT[Btn8L])
			targetRPM = 2500;
		else if (vexRT[Btn8U])
			targetRPM = 2700;
	}
}
