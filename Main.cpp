#include <cstdint>
#include "WPILib.h"
#include <CANTalon.h>
#include <Timer.h>
#include <iostream>
#include <string>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/core/types.hpp>

using namespace std;

class Robot: public IterativeRobot
{
	Joystick joystickMain; // Drive
	Joystick joystickSecondary; //Mechanisms

	CANTalon FL;
	CANTalon FR;
	CANTalon RL;
	CANTalon RR;


	CANTalon ballMotor;
	CANTalon ropeMotor;

	double rampRate = 8.2; // Higher = Faster Acceleration

	double speedRatio = 2; //Ratio used to control speed

	double magnitude; // joystick angles

	double rightTrigger = 0.0;
	double leftTrigger = 0.0;

	double ropeRatio = 2;


	DoubleSolenoid *gearRampSole = new DoubleSolenoid(7 ,6);
	DoubleSolenoid *gearSole = new DoubleSolenoid(4 ,5);
	DoubleSolenoid *gearDoorSole = new DoubleSolenoid(1 ,0);


	Timer *timer = new Timer();

	double rightMag, leftMag;
	const double magRatio = 0.93;

	double rightTalonRatio = 0.965;

//	Compressor *c = new Compressor(0);
	double turningRatio = 0.485;

	double zAxisRatio = 2; // makes turning easier

	bool doGearStuffs = false;

	cs::UsbCamera camera1;
	cs::UsbCamera camera2;
	cs::VideoSink server;


	double dist = 4.0;

	bool doAuto = true;

	Preferences *prefs;
	ADXRS450_Gyro gyro;
	float setOffset;
	bool setRightAngle = true;

	bool resetTimerOnce = true;

	string autoMode = "3"; // 0 is left 1 is middle and 2 is right

public:
	Robot() :
		joystickMain(0),
		joystickSecondary(1),

		FL(1), // R -1
		FR(2), // R -1

		RL(4), // L -1
		RR(3), // L -1
		ballMotor(5),
		ropeMotor(6)
	{

		prefs = Preferences::GetInstance();



		FL.SetVoltageRampRate(rampRate);
		FR.SetVoltageRampRate(rampRate);
		RL.SetVoltageRampRate(rampRate);
		RR.SetVoltageRampRate(rampRate);

		ballMotor.SetVoltageRampRate(9.0);
		ropeMotor.SetVoltageRampRate(9.0);

	}

private:

	void RobotInit()
	{
		CameraServer::GetInstance()->StartAutomaticCapture();
		CameraServer::GetInstance()->SetSize(CameraServer::GetInstance()->kSize320x240);



		timer->Start();
   		timer->Reset();
//   		setBallMechanismUp();
	}

	void AutonomousInit()
	{
		setRightAngle = true;
		autoMode = prefs->GetString("Auto Mode","3");
		gyro.Calibrate();

		resetTimerOnce = true;
		doGearStuffs = true;

		lockGearDoor();
	}

	void AutonomousPeriodic()
	{
		if(autoMode == "0") {
			goForwardAutonomous();
		}
		else if(autoMode == "1"){
			middlePegAutonomous();
		}
		else if(autoMode == "2") {
			goForwardAutonomous();
		}
		else if(autoMode == "4") {
			autonomousTest();
		}
	}

	void gyroTest()
	{
		double offsetAngle = gyro.GetAngle();
		cout << offsetAngle << endl;
		float maxOffset = 2.0f;




		if (offsetAngle > maxOffset)
		{
			setRight(-0.1);
			setLeft(0.1);
			return;
		}
		else if(offsetAngle < -maxOffset) {
			setRight(0.1);
			setLeft(-0.1);
			return;
		}

//		if (joystickMain.GetRawButton(7))
//			setRightAngle = true;
//
//		if (joystickMain.GetRawButton(8))
//			setRightAngle = false;
		setRight(0);
		setLeft(0);

	}

	void autonomousTest()
	{
		if (resetTimerOnce) // resets timer
		{
			timer->Reset();
			resetTimerOnce = false;
		}

		float distanceTime = 1.98;                       // move to the peg
		float fixOffsetTime = distanceTime + 0.3;        // check being off a bit
		float openGearDoorTime = distanceTime + 0.5;     // open gear door
		float pushGearOutTime = distanceTime + 1.0;      // push gear out
		float pullGearInTime = distanceTime + 1.7;       // pull gear in
		float finishDroppingGearTime = distanceTime + 3; // move backwards and close gear door at the end
		float finishGearTime = distanceTime + 3.2;       // finish all the gear stuff and be ready to turn to go
		float turn45Time = distanceTime + 3.5;           // turn 45 degrees

		// moves ahead towards the peg
		if (timer->Get() < distanceTime)
		{
			double offsetAngle = gyro.GetAngle();
			cout << offsetAngle << "   " << timer->Get() << endl;
			float maxOffset = 2.0f;

			float baseValue = -0.3f;
			float adjustment = 0.07f;

			if(offsetAngle > maxOffset)
			{
				setRight(-adjustment + baseValue);
				setLeft(adjustment + baseValue);
				return;
			}
			else if(offsetAngle < -maxOffset)
			{
				setRight(adjustment + baseValue);
				setLeft(-adjustment + baseValue);
				return;
			}
			else
			{
				setRight(baseValue);
				setLeft(baseValue);
				return;
			}
			setRight(0);
			setLeft(0);
		}

		setRight(0);
		setLeft(0);


		if (timer->Get() < fixOffsetTime) // makes sure the robot is facing the peg
		{
			gyroTest();
			return;
		}

		if (timer->Get() < openGearDoorTime) // open gear door
		{
			openGearDoor();
			return;
		}


		if (doGearStuffs && timer->Get() < finishGearTime)
		{
			if (timer->Get() > finishDroppingGearTime)
			{
				lockGearDoor();
				setRight(0);
				setLeft(0);
				doGearStuffs = false;
				return;
			}
			else if (timer->Get() >= pullGearInTime)
			{
				pullGearIn();
				setRight(0.2);
				setLeft(0.2);
				return;
			}
			else if (timer->Get() >= pushGearOutTime)
			{
				setRight(0);
				setLeft(0);
				pushGearOut();
				return;
			}
		}

		setRight(0);
		setLeft(0);

		if (timer->Get() < turn45Time && timer->Get() > finishGearTime)
		{
			turn60();
			return;
		}

		setRight(0);
		setLeft(0);
	}


	void setAngle(int angle) // makes sure the gyro is in a range of maxOffset away from "angle"
	{
		double offsetAngle = gyro.GetAngle();
		cout << offsetAngle << endl;
		float maxOffset = 10.0f;
		float smlMaxOffset = 4.0f;

		if (offsetAngle > angle + maxOffset)
		{
			setRight(-0.25);
			setLeft(0.25);
			return;
		}
		else if(offsetAngle < angle -maxOffset) {
			setRight(0.25);
			setLeft(-0.25);
			return;
		}
		else if (offsetAngle > angle + smlMaxOffset)
		{
			setRight(-0.15);
			setLeft(0.15);
			return;
		}
		else if (offsetAngle < angle - smlMaxOffset)
		{
			setRight(0.15);
			setLeft(-0.15);
			return;
		}
		setRight(0);
		setLeft(0);
	}

	void turn60() // turn 60 degrees and fix offsets
	{

		int angle = 60;
		double offsetAngle = gyro.GetAngle();
		cout << offsetAngle << endl;
		float bigMaxOffset = 10.0f;
		float smlMaxOffset = 4.0f;



		if (offsetAngle > angle + bigMaxOffset)
		{
			setRight(-0.2);
			setLeft(0.2);
			return;
		}
		else if(offsetAngle < angle -bigMaxOffset) {
			setRight(0.2);
			setLeft(-0.2);
			return;
		}

		else if (offsetAngle > angle + smlMaxOffset)
		{
			setRight(-0.15);
			setLeft(0.15);
			return;
		}
		else if (offsetAngle < angle - smlMaxOffset)
		{
			setRight(0.15);
			setLeft(-0.15);
			return;
		}
		setRight(0);
		setLeft(0);

	}

	void goForwardAutonomous() {
		if (setRightAngle)
		{
			timer->Reset();
			setRightAngle = false;
		}


		float distanceTime = 2.2;


		if (timer->Get() < distanceTime)
		{
			double offsetAngle = gyro.GetAngle();
			cout << offsetAngle << "   " << timer->Get() << endl;
			float maxOffset = 2.0f;

			float baseValue = -0.3f;
			float adjustment = 0.07f;

			if(offsetAngle > maxOffset)
			{
				setRight(-adjustment + baseValue);
				setLeft(adjustment + baseValue);
				return;
			}
			else if(offsetAngle < -maxOffset)
			{
				setRight(adjustment + baseValue);
				setLeft(-adjustment + baseValue);
				return;
			}
			else
			{
				setRight(baseValue);
				setLeft(baseValue);
				return;
			}

			setRight(0);
			setLeft(0);
		}

		setRight(0);
		setLeft(0);

	}

	void middlePegAutonomous() {

		if (setRightAngle)
		{
			timer->Reset();
			setRightAngle = false;
		}




		float distanceTime = 1.98;
		float fixOffsetTime = distanceTime + 0.3;
		float openGearDoorTime = distanceTime + 0.5;
		float finishDroppingGearTime = distanceTime + 7;
		float pullGearInTime = distanceTime + 1.7;
		float pushGearOutTime = distanceTime + 1.0;
		float turnToGoTime = distanceTime + 7.8;


		if (timer->Get() < distanceTime)
		{


			double offsetAngle = gyro.GetAngle();
			cout << offsetAngle << "   " << timer->Get() << endl;
			float maxOffset = 2.0f;

			float baseValue = -0.3f;
			float adjustment = 0.07f;

			if(offsetAngle > maxOffset)
			{
				setRight(-adjustment + baseValue);
				setLeft(adjustment + baseValue);
				return;
			}
			else if(offsetAngle < -maxOffset)
			{
				setRight(adjustment + baseValue);
				setLeft(-adjustment + baseValue);
				return;
			}
			else
			{
				setRight(baseValue);
				setLeft(baseValue);
				return;
			}




			setRight(0);
			setLeft(0);
		}

		setRight(0);
		setLeft(0);


		if (timer->Get() < fixOffsetTime)
		{
			gyroTest();
			return;
		}

		if (timer->Get() < openGearDoorTime)
			openGearDoor();


		if (doGearStuffs)
		{
			if (timer->Get() > finishDroppingGearTime)
			{
				lockGearDoor();
				doGearStuffs = false;
				return;
			}
			else if (timer->Get() >= pullGearInTime)
			{
				pullGearIn();
				setRight(0.2);
				setLeft(0.2);
				return;
			}
			else if (timer->Get() >= pushGearOutTime)
			{
				setRight(0);
				setLeft(0);
				pushGearOut();
				return;
			}
		}

	}


	void TeleopInit()
	{
	}


	void TeleopPeriodic()
	{
		joystickInputsToDriving();
		ropeMechanism();
		newGearMechanism();
		mainJoystickHandyButtons();
	}

	void mainJoystickHandyButtons()
	{
		if (joystickMain.GetRawButton(11))
		{ // rotate to right
			setLeft(0.2);
			setRight(-0.2);
			spendTime(0.2);
			dontMove();
		}

		if (joystickMain.GetRawButton(12))
		{ // rotate to left
			setLeft(-0.2);
			setRight(0.2);
			spendTime(0.2);
			dontMove();
		}

		if (joystickMain.GetRawButton(10))
		{ // go a bit forward
			setLeft(-0.1);
			setRight(-0.1);
			spendTime(0.2);
			dontMove();
		}
	}

	// Set the speed of the motors on the right side
	void setRight(double value)
	{
		FL.Set(value);
		FR.Set(value);
	}

	// Combining both left motors to set speed to value
	void setLeft(double value)
	{
		RR.Set(-value);
		RL.Set(-value);
	}


	void getTriggers()
	{
		rightTrigger = joystickSecondary.GetRawAxis(3);
		leftTrigger = joystickSecondary.GetRawAxis(2);
	}

	void ropeMechanism()
	{
		getTriggers();

		if (joystickSecondary.GetRawButton(7))
		{
			ropeRatio = 1;
		}
		else
			ropeRatio = 2;

		if (rightTrigger > 0.3)
		{ //Used for climbing up
			ropeMotor.Set(rightTrigger/ropeRatio); //Maybe opposite
		}
		else if (leftTrigger > 0.3)
		{ //Used for climbing down
			ropeMotor.Set(-leftTrigger/4);
		}
		else
		{
			ropeMotor.Set(0.0);
		}
	}

	void pushGearOut()
	{
		gearSole->Set(DoubleSolenoid::Value::kForward);
	}

	void pullGearIn()
	{
		gearSole->Set(DoubleSolenoid::Value::kReverse);
	}

	void openGearDoor()
	{
		gearDoorSole->Set(DoubleSolenoid::Value::kForward);
	}

	void lockGearDoor()
	{
		gearDoorSole->Set(DoubleSolenoid::Value::kReverse);
	}


	void spendTime(float seconds)
	{
		timer->Reset();
		while (1)
		{
			if (timer->Get() >= seconds)
			{
				return;
			}
		}
	}

	void newGearMechanism()
	{
		if(joystickSecondary.GetRawButton(3)) {
			gearRampSole->Set(DoubleSolenoid::Value::kForward);
		}
		if(joystickSecondary.GetRawButton(2)) {
			gearRampSole->Set(DoubleSolenoid::Value::kReverse);
		}


		if (joystickSecondary.GetRawButton(5) && !doGearStuffs)
		{
			doGearStuffs = true;
			timer->Reset();
			openGearDoor();
		}
		if (doGearStuffs)
		{
			if (timer->Get() > 7)
			{
				lockGearDoor();
				doGearStuffs = false;
			}
			else if (timer->Get() >= 1.4)
				pullGearIn();
			else if (timer->Get() >= 0.7)
				pushGearOut();
		}
	}


	// Set speed ratio using buttons
	void setRatio()
	{
		speedRatio = 1.1; // neutral speed 2
		zAxisRatio = 2; // 3
		if (joystickMain.GetRawButton(7))
		{
			speedRatio = 1; // fast speed
			zAxisRatio = 3; // 4
		}
		else if (joystickMain.GetRawButton(9))
		{
			speedRatio = 3; // slow speed 6
			zAxisRatio = 4; // 6
		}
	}


	void dontMove()
	{
		setLeft(0.0);
		setRight(0.0);
	}

	void zAxisDrive()
	{
		double realMag = joystickMain.GetMagnitude();//Get real magnitude
		magnitude = realMag/speedRatio; // Get magnitude adjusted by speedRatio [0, 1]

		leftMag = magnitude;
		rightMag = magnitude;

		double zAxis = joystickMain.GetRawAxis(2);

		double rotation = joystickMain.GetDirectionDegrees(); //Get rotation values from joystick [-180, 180]

		if(realMag > 0.2)
		{
			if (rotation >= -60 && rotation  <= 60)
			{
				if(zAxis > 0.4)
				{
					rightMag = rightMag - zAxis/zAxisRatio;
				}
				else if (zAxis < -0.3)
				{
					leftMag = leftMag + zAxis/zAxisRatio;
				}
				setLeft(-leftMag);
				setRight(-rightMag);
			}
			else if ((rotation <= -120 && rotation >= -180) || (rotation >= 120 && rotation <= 180))
			{
				if(zAxis > 0.4)
				{
					rightMag = rightMag + zAxis/zAxisRatio;
				}
				else if (zAxis < -0.3)
				{
					leftMag = leftMag - zAxis/zAxisRatio;
				}
				setLeft(leftMag);
				setRight(rightMag);
			}
		}

		else if (realMag <= 0.4)
		{
			if(zAxis > 0.2)
			{
				setRight(zAxis/1.7);
				setLeft(-zAxis/1.7);
			}
			else if (zAxis < -0.2)
			{
				setRight(zAxis/1.7);
				setLeft(-zAxis/1.7);
			}
			else
			{
				dontMove();
			}
		}
		else
		{
			dontMove();
		}
	}

	void joystickInputsToDriving()
	{
		setRatio();
		zAxisDrive();
	}


	void TestPeriodic()
	{

	}
};

START_ROBOT_CLASS(Robot);
