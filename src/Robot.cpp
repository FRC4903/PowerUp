#include <string>
#include <bits/stdc++.h>

#include <IterativeRobot.h>
#include <LiveWindow/LiveWindow.h>
#include <SmartDashboard/SendableChooser.h>
#include <SmartDashboard/SmartDashboard.h>
#include <Ultrasonic.h>
#include "ctre/Phoenix.h"
//#include "WPILib.h"
//#include "Phoenix.h"
#include "DoubleSolenoid.h"
#include "ADXRS450_Gyro.h"
#include "DriverStation.h"
#include <Timer.h>
#include "Preferences.h"
#include "Joystick.h"
#include "AnalogInput.h"
//#include <opencv2>
#include "CameraServer.h"
#include "Servo.h"


using namespace frc;
using namespace std;





class Robot : public frc::IterativeRobot {

public:
	Timer *timer = new Timer();
	const double LIFT_CONSTANT_COEFFICIENT = 10;

	Ultrasonic *ultraFront;
	TalonSRX talonRight1, talonRight2, talonLeft1, talonLeft2;


	TalonSRX *cubeLiftMotor;

	TalonSRX hookTalon1;
	TalonSRX hookTalon2;

	AnalogInput cubeLiftInductiveTop;
	AnalogInput cubeLiftInductiveBottom;

	AnalogInput hookLiftInductiveTop;
	AnalogInput hookLiftInductiveBottom;

	ADXRS450_Gyro gyro;

	double rightEncoderConstant, leftEncoderConstant;

	TalonSRX cubeIntakeTalonLeft, cubeIntakeTalonRight;
	DoubleSolenoid *cubeArmTiltSole = new DoubleSolenoid(6 ,1);

	bool topInductiveSensor, bottomInductiveSensor;
	bool topHookInductiveSensor, bottomHookInductiveSensor;

	bool initialValueSet = false;
	double initialLiftPosition = 0;

	double leftEncoderInitial = 0.0;
	double rightEncoderInitial = 0.0;

	int kTimeoutMs = 10;
	int kPIDLoopIdx = 0;
	int kSlotIdx = 0;

	double initialValTalon;
	Joystick joystickMain;
	Joystick joystickMechanisms;
	double lefty;

	double j_x, j_y;
	double moderator;

	double beginningDiff;

	string gameData;
	Preferences* preferences;
//	DriverStation* driveStation;

	bool done = false;

	Timer *autoTimer = new Timer();


	Servo *rightServo;
	Servo *leftServo;



	// SETUP SECTION
	//
	//
	//

	Robot() : talonRight1(1), talonRight2(2),
			talonLeft1(4), talonLeft2(5),
			cubeIntakeTalonLeft(7),cubeIntakeTalonRight(8),
			joystickMain(0),
			joystickMechanisms(1),
			cubeLiftInductiveTop(0),cubeLiftInductiveBottom(1),hookTalon1(9),hookTalon2(10),hookLiftInductiveTop(2),hookLiftInductiveBottom(3),
			rightServo(), leftServo()
	{
		cubeLiftMotor = new TalonSRX(6);
		preferences = Preferences::GetInstance();
//		driverStation = DriverStation::GetInstance();
	}

	void setup()
	{
		beginningDiff = -1000000;
		moderator = 0.5;
		timer->Start();

	}

	void RobotInit() {

		ultraFront = new Ultrasonic(5, 4);
		ultraFront->SetAutomaticMode(true);
		talonRight1.SetInverted(true);
		talonRight2.SetInverted(true);

		autoTimer->Start();


		cubeArmTiltSole->Set(DoubleSolenoid::kReverse);


		gyro.Calibrate();
		gyro.Reset();

		cubeIntakeTalonLeft.SetNeutralMode(NeutralMode::Brake);
		cubeIntakeTalonRight.SetNeutralMode(NeutralMode::Brake);

		cubeIntakeTalonLeft.SetInverted(true);

		cubeLiftMotor->SetNeutralMode(NeutralMode::Brake);


		talonRight1.SetNeutralMode(NeutralMode::Coast);
		talonRight2.SetNeutralMode(NeutralMode::Coast);
		talonLeft1.SetNeutralMode(NeutralMode::Coast);
		talonLeft2.SetNeutralMode(NeutralMode::Coast);

		hookTalon1.SetNeutralMode(NeutralMode::Brake);
		hookTalon2.SetNeutralMode(NeutralMode::Brake);

		setupEncoderTalon(&talonLeft2);
		setupEncoderTalon(&talonRight1);

		timer->Start();

		CameraServer::GetInstance()->StartAutomaticCapture();
//		cs::UsbCamera camera = CameraServer


	}

	void setupEncoderTalon(TalonSRX* talon) {
		int absPos = talon->GetSelectedSensorPosition(0) & 0xFFF;
		talon->SetSelectedSensorPosition(absPos, kPIDLoopIdx, kTimeoutMs);

		talon->ConfigSelectedFeedbackSensor(FeedbackDevice::CTRE_MagEncoder_Relative, kPIDLoopIdx, kTimeoutMs);
		talon->SetSensorPhase(true);
		talon->ConfigNominalOutputForward(0, kTimeoutMs);
		talon->ConfigNominalOutputReverse(0, kTimeoutMs);
		talon->ConfigPeakOutputForward(1, kTimeoutMs);
		talon->ConfigPeakOutputReverse(-1, kTimeoutMs);
		talon->Config_kF(kPIDLoopIdx, 0.0, kTimeoutMs);
		talon->Config_kP(kPIDLoopIdx, 0.1, kTimeoutMs);
		talon->Config_kI(kPIDLoopIdx, 0.0, kTimeoutMs);
		talon->Config_kD(kPIDLoopIdx, 0.0, kTimeoutMs);
}








	// Autonomous SECTION
		//
		//
		//
		//
		//
		//
		//
		//
		//
		//

	void TestInit() {

	}


	void TestPeriodic() {

	}

	void AutonomousInit() {
		gyro.Reset();

		gameData = DriverStation::GetInstance().GetGameSpecificMessage();
		setCubeArmTilt(false);

		driveSystemCoastMode(false);
		done= false;

		autoTimer->Reset();

		rightServo->SetAngle(rightServo->GetAngle() + 90);
		leftServo->SetAngle(leftServo->GetAngle() - 90);
	}

	void autoCubeDown() {

		getInductiveSensors();
		while(!bottomInductiveSensor && autoTimer->Get() < 15) {
			getInductiveSensors();
			cubeLiftMotor->Set(ControlMode::PercentOutput,-1.0);
		}
		cubeLiftMotor->Set(ControlMode::PercentOutput,0.0);
	}

	void autoCubeUp() {
		getInductiveSensors();
		while(!topInductiveSensor && autoTimer->Get() < 15) {
			getInductiveSensors();
			cubeLiftMotor->Set(ControlMode::PercentOutput,1.0);
		}
		cubeLiftMotor->Set(ControlMode::PercentOutput,0.0);
	}

	void autoCubeIn() {
		while(ultraFront->GetRangeInches() > 10 && autoTimer->Get() < 15) {
			setCubeMotors(-0.8);
		}
		setCubeMotors(0.0);
	}

	void goForwardAndUp(int inchesInt, double speed=0.5)
	{
		double inches = double(inchesInt);
		const double encoderConst = 35.34; // Ratio adjustment from encoder ticks to inches

//		const double a = 0.023944549;
//		const double b = 35.24782609/2.0;

//		const double a = 0.9467455621;
//		const double b = -2.840236686;
//		const double c = 42.130177514;

		double initRight = talonRight1.GetSelectedSensorPosition(kPIDLoopIdx);
		double initLeft  = talonLeft2.GetSelectedSensorPosition(kPIDLoopIdx);

		double rightside = speed;
		double leftside  = speed;
		setRight(rightside);
		setLeft(leftside);
//		(inches*inches * a) + b*inches + c
		const double a = 31.66666666666;
		double b = -265.0;

		if(inches >= 200)
		{
			b = 200.0;
		}
		else if(inches >= 100) {
			 b = 0.0;
		}


		double distance = inches*a + b;

		if (inches > 137 && inches < 143)
		{
			distance = 4450;
		}
		else if (inches > 69 && inches < 71)
		{
			distance = 2000;
		}
		else if (inches > 196 && inches < 201)
		{
			distance = 6756;
		}
		else if (inches > 219 && inches < 225)
		{
			distance = 8498;
		}

		while (distance > -(initLeft - talonLeft2.GetSelectedSensorPosition(kPIDLoopIdx))  && autoTimer->Get() < 15)
		{
			getInductiveSensors();
			if (!topInductiveSensor) {
				cubeLiftMotor->Set(ControlMode::PercentOutput,1.0);
			}
			else
			{
				cubeLiftMotor->Set(ControlMode::PercentOutput,0.0);
			}
			//0.2 before
			rightside = speed;
			leftside = speed;
			double coeff = 0.004;
			setRight(rightside);
			setLeft(leftside);
		}
		setRight(0.0);
		setLeft(0.0);
		autoCubeUp();
	}



	void goBackAndBringDown(double inches, double speed=0.4)
	{
		const double encoderConst = 35.34; // Ratio adjustment from encoder ticks to inches
		const double a = 0.023944549;
		const double b = 35.24782609;
		double initRight = talonRight1.GetSelectedSensorPosition(kPIDLoopIdx);
		double initLeft  = talonLeft2.GetSelectedSensorPosition(kPIDLoopIdx);
		double rightside = -speed;
		double leftside  = -speed;
		setRight(rightside);
		setLeft(leftside);

		while (b*inches > (initLeft - talonLeft2.GetSelectedSensorPosition(kPIDLoopIdx)) && autoTimer->Get() < 15)
		{
			getInductiveSensors();
			if (!bottomInductiveSensor) {
				cubeLiftMotor->Set(ControlMode::PercentOutput,-1.0);
			}
			else
			{
				cubeLiftMotor->Set(ControlMode::PercentOutput,0.0);
			}
			//0.2 before
			rightside = -speed;
			leftside = -speed;
			double coeff = 0.004;
			setRight(rightside);
			setLeft(leftside);
		}
		setRight(0.0);
		setLeft(0.0);
		autoCubeDown();
	}

	void goBackAndBringUp(double inches, double speed=0.4)
		{
			const double encoderConst = 35.34; // Ratio adjustment from encoder ticks to inches
			const double a = 0.023944549;
			const double b = 35.24782609;
			double initRight = talonRight1.GetSelectedSensorPosition(kPIDLoopIdx);
			double initLeft  = talonLeft2.GetSelectedSensorPosition(kPIDLoopIdx);
			double rightside = -speed;
			double leftside  = -speed;
			setRight(rightside);
			setLeft(leftside);

			while (b*inches > (initLeft - talonLeft2.GetSelectedSensorPosition(kPIDLoopIdx)) && autoTimer->Get() < 15)
			{
				getInductiveSensors();
				if (!topInductiveSensor) {
					cubeLiftMotor->Set(ControlMode::PercentOutput,1.0);
				}
				else
				{
					cubeLiftMotor->Set(ControlMode::PercentOutput,0.0);
				}
				//0.2 before
				rightside = -speed;
				leftside = -speed;
				double coeff = 0.004;
				setRight(rightside);
				setLeft(leftside);
			}
			setRight(0.0);
			setLeft(0.0);
			autoCubeUp();
		}


	void AutonomousPeriodic() {
		if(gameData.length() > 0 && !done)
		{
			done = true;
			int pos = preferences->GetInt("autoPos", 0); // 1 left 2 center 3 right
			int endGame = preferences->GetInt("endGame", 0); // 0 - Vault, 1 - Switch, 2 - farSwitch

			char switchPos = gameData[0];
			char scalePos = gameData[1];
			char farSwitch = gameData[2];

			setCubeArmTilt(true);

			if (pos == 4)
			{
				goForwardInInches(140);
			}

			if(pos == 1) { // left
				if(switchPos == 'L') {
					goForwardInInches(140);
					turn(90);
					forwardUltrasonic(0.3, 18);
					shootCubeOutAuto(0.3);

					goBackInInches(15);
					autoCubeDown();
				}
				else if (switchPos == 'R') {
					goForwardInInches(140);

					if (endGame == 0) {
						goBackAndBringDown(60);
						turn(90);
					}
				}
			}
			else if(pos == 2) { // center
				if(switchPos == 'L') { // tested!
					goForwardInInches(15);
					turn(-50);
					goForwardInInches(85);
					turn(50);

					forwardUltrasonic();
					setRight(0.0);
					setLeft(0.0);
					shootCubeOutAuto();

					goBackAndBringDown(15);
					turn(-50);
					goBackInInches(65);
					turn(50);

					ultraTakeInMoveForward();

					if (endGame == 0) {
						turn(180);
					}
					else if (endGame == 1) {
						goBackAndBringUp(15);
						turn(-45);
					}
					else if (endGame == 2) {
						goBackAndBringUp(15);
						if(farSwitch == 'L') {
							turn(-90);
						}
						else {
							turn(90);
						}
					}

				}
				else if (switchPos == 'R') {
					goForwardInInches(15);
					turn(45);
					goForwardInInches(73);
					turn(-45);
					forwardUltrasonic();

					setRight(0.0);
					setLeft(0.0);
					shootCubeOutAuto();

					goBackAndBringDown(15);
					turn(45);
					goBackInInches(60);
					turn(-45);

					ultraTakeInMoveForward();

					if (endGame == 0) {
						turn(180);
					}
					else if (endGame == 1) {
						goBackAndBringUp(15);
						turn(45);
					}
					else if (endGame == 2) {
						goBackAndBringUp(15);
						if(farSwitch == 'L') {
							turn(-90);
						}
						else {
							turn(90);
						}
					}

				}
			}
			else if(pos == 3) { // right
				if(switchPos == 'L') {
					goForwardInInches(140);

					if (endGame == 0) {
						goBackAndBringDown(60);
						turn(-90);
					}
				}
				else if (switchPos == 'R') {
					goForwardInInches(140);
					turn(-90);
					forwardUltrasonic(0.3, 18);
					shootCubeOutAuto(0.3);

					goBackInInches(15);
					autoCubeDown();
				}
			}

			setRight(0.0);
			setLeft(0.0);
			setCubeMotors(0.0);
		}
	}

	void shootCubeOutAuto(double speed=0.4) {
		setCubeMotors(speed);
		double start = timer->Get();
		while(timer->Get() - start < 1 && autoTimer->Get() < 15) {
			int a = 0;
		}

		setCubeMotors(0.0);
	}

	void resetEncoder() {
		rightEncoderInitial = talonLeft2.GetSelectedSensorPosition(kPIDLoopIdx);
		leftEncoderInitial = talonLeft2.GetSelectedSensorPosition(kPIDLoopIdx);
	}

	double getLeftEncoderValue() {
		return (talonRight1.GetSelectedSensorPosition(kPIDLoopIdx) - leftEncoderInitial) / leftEncoderConstant;
	}

	double getRightEncoderValue() {
		return (talonLeft2.GetSelectedSensorPosition(kPIDLoopIdx) - rightEncoderInitial) / rightEncoderConstant;
	}

	void driveSystemCoastMode(bool a)
	{
		if (a)
		{
			talonRight1.SetNeutralMode(NeutralMode::Coast);
			talonRight2.SetNeutralMode(NeutralMode::Coast);
			talonLeft1.SetNeutralMode(NeutralMode::Coast);
			talonLeft2.SetNeutralMode(NeutralMode::Coast);
		}
		else {
			talonRight1.SetNeutralMode(NeutralMode::Brake);
			talonRight2.SetNeutralMode(NeutralMode::Brake);
			talonLeft1.SetNeutralMode(NeutralMode::Brake);
			talonLeft2.SetNeutralMode(NeutralMode::Brake);
		}
	}

	void goForwardInInches(int inchesInt, double speed = 0.5)
	{
		double inches = double(inchesInt);
		const double encoderConst = 35.34; // Ratio adjustment from encoder ticks to inches

//		const double a = 0.023944549;
//		const double b = 35.24782609/2.0;

//		const double a = 0.9467455621;
//		const double b = -2.840236686;
//		const double c = 42.130177514;


		double initRight = talonRight1.GetSelectedSensorPosition(kPIDLoopIdx);
		double initLeft  = talonLeft2.GetSelectedSensorPosition(kPIDLoopIdx);
		double rightside = speed;
		double leftside  = speed;
		setRight(rightside);
		setLeft(leftside);
//		(inches*inches * a) + b*inches + c
		const double a = 31.66666666666;
		double b = -265.0;

		if(inches >= 200)
		{
			b = 200.0;
		}
		else if(inches >= 100) {
			 b = 0.0;
		}


		double distance = inches*a + b;

		if (inches > 137 && inches < 143)
		{
			distance = 4450;
		}
		else if (inches > 69 && inches < 71)
		{
			distance = 2000;
		}
		else if (inches > 196 && inches < 201)
		{
			distance = 6100;
		}
		else if (inches > 219 && inches < 225)
		{
			distance = 7450;
		}
		else if (inches > 249 && inches < 251)
		{
			distance = 7800;
		}

		while (distance > -(initLeft - talonLeft2.GetSelectedSensorPosition(kPIDLoopIdx)) && autoTimer->Get() < 15)
		{
			//0.2 before
			rightside = speed;
			leftside = speed;
			double coeff = 0.004;
			setRight(rightside);
			setLeft(leftside);
		}
		setRight(0.0);
		setLeft(0.0);
	}

	void goBackInInches(double inches, double speed = 0.4)
		{

			const double encoderConst = 35.34; // Ratio adjustment from encoder ticks to inches
			const double a = 0.023944549;
			const double b = 35.24782609;
			double initRight = talonRight1.GetSelectedSensorPosition(kPIDLoopIdx);
			double initLeft  = talonLeft2.GetSelectedSensorPosition(kPIDLoopIdx);
			double rightside = -speed;
			double leftside  = -speed;
			setRight(rightside);
			setLeft(leftside);

			while (b*inches > (initLeft - talonLeft2.GetSelectedSensorPosition(kPIDLoopIdx)) && autoTimer->Get() < 15)
			{
				//0.2 before
				rightside = -speed;
				leftside = -speed;
				double coeff = 0.004;
				setRight(rightside);
				setLeft(leftside);
			}
			setRight(0.0);
			setLeft(0.0);
		}

	void forwardUltrasonic(double halfSpeed = 0.30, double dist = 28)
	{
		double initDist = ultraFront->GetRangeInches();
		int counter = 0;
		while (ultraFront->GetRangeInches() > dist && autoTimer->Get() < 15)
		{
			if (counter < 20)
			{
				initDist = initDist * counter + ultraFront->GetRangeInches();
				counter++;
				initDist /= counter;
			}
			double ultraVal = ultraFront->GetRangeInches();
			double speed = halfSpeed + halfSpeed * ((ultraVal > initDist ? initDist : ultraVal) / initDist);
			setRight(speed);
			setLeft(speed);
		}
		setRight(0.0);
		setLeft(0.0);
	}

	void ultraTakeInMoveForward(double halfSpeed = 0.1, double dist = 12)
	{
		double initDist = ultraFront->GetRangeInches();
		int counter = 0;
		while (ultraFront->GetRangeInches() > dist && autoTimer->Get() < 15)
		{
			setCubeMotors(-0.6);
			if (counter < 20)
			{
				initDist = initDist * counter + ultraFront->GetRangeInches();
				counter++;
				initDist /= counter;
			}
			double ultraVal = ultraFront->GetRangeInches();
			double speed = halfSpeed + halfSpeed * ((ultraVal > initDist ? initDist : ultraVal) / initDist);
			setRight(speed);
			setLeft(speed);
		}
		setCubeMotors(0.0);
		setRight(0.0);
		setLeft(0.0);
	}

	double convertAngle(double val)
	{
		return (double)(val - initialValTalon)/2000*360;
	}

	void turn(int angleInt,double initial=0.6)
		{
			double angle = double(angleInt);
			gyro.Reset();
			if (angle > 0)
			{
				while (gyro.GetAngle() < angle && autoTimer->Get() < 15)
				{
					double speed = initial + initial * (1 -  gyro.GetAngle() / angle)/2;
					setRight(-speed);
					setLeft(speed);
				}
			}
			else if (angle < 0)
			{
				while (gyro.GetAngle() > angle && autoTimer->Get() < 15)
				{
					double speed = initial + initial * (1 - abs(gyro.GetAngle() / angle))/2;
					setRight(speed);
					setLeft(-speed);
				}
			}
			stopDriveMotors();
		}









	// TELEOP SECTION
	//
	//
	//
	//
	//
	//
	//
	//
	//
	//

	void TeleopInit() {
		driveSystemCoastMode(true);  //DRIVING

		gyro.Reset();
		timer->Start();
		timer->Reset();

		reverseDrive = false;

		lefty  = talonLeft2.GetSelectedSensorPosition(kPIDLoopIdx);

	}

	void TeleopPeriodic() {
		getInductiveSensors();      //uncomment to drive
		driveSystem();      //uncomment to drive
		mechanismSystem(); //uncomment to drive

//		if(joystickMechanisms.GetRawButton(1)) {
//				talonLeft1.Set(ControlMode::PercentOutput,0.2);
//				talonLeft2.Set(ControlMode::PercentOutput,0.0);
//			}
//			else if(joystickMechanisms.GetRawButton(2)) {
//				talonLeft1.Set(ControlMode::PercentOutput,0.0);
//				talonLeft2.Set(ControlMode::PercentOutput,0.2);
//			}
//			else {
//				talonLeft1.Set(ControlMode::PercentOutput,0.0);
//				talonLeft2.Set(ControlMode::PercentOutput,0.0);
//			}

//		driveSystemCoastMode(false);

//		double amount = preferences->GetDouble("forwardAmount");
//
//		if(joystickMain.GetRawButton(1)) {
//			goForwardInInches(6600);
//		}

//		if (joystickMain.GetRawButton(8))
//		{
//			autoTimer->Reset();
//			forwardUltrasonic();
//		}
	}






	// DRIVE SYSTEM

	bool reverseDrive = false;

	void driveSystem()
	{

		if (joystickMain.GetRawButton(1)) // if green a button is pressed
			moderator =1.0; // makes robot go faster .. 1.0 for carpet
		else if (joystickMain.GetRawButton(2)) // if red b button is pressed
			moderator = 0.3; // make it really slow
		else // base case let it be half speed
			moderator = 0.85; // limits the range given from the controller // 0.85 for carpet

		j_x = joystickMain.GetRawAxis(1) * moderator;
		j_y = joystickMain.GetRawAxis(0) * moderator;

		if((j_x < 0 && j_x >= -0.05) || (j_x > 0 && j_x <= 0.05)) {
			j_x = 0;
		}

		if((j_y < 0 && j_y >= -0.05) || (j_y > 0 && j_y <= 0.05)) {
			j_y = 0;
		}




		reverseDrive = joystickMain.GetRawButton(5);

		double speedL = +j_y - j_x;
		double speedR = -j_y - j_x;

		if (reverseDrive) {
			speedL = -j_y + j_x;
			speedR = +j_y + j_x;
		}

		setLeft(speedL);
		setRight(speedR);


	}


	void setRight(double value)
	{
		talonRight1.Set(ControlMode::PercentOutput, value);
		talonRight2.Set(ControlMode::PercentOutput, value);
	}

	void setLeft(double value)
	{
		talonLeft1.Set(ControlMode::PercentOutput, value);
		talonLeft2.Set(ControlMode::PercentOutput, value);
	}

	void stopDriveMotors()
	{
		setRight(0.0);
		setLeft(0.0);
	}






	// MECHANISMS
	//
	//

	void mechanismSystem()
	{
		cubeMechanism();
		hookLiftMechanism();
	}



	//  Mechanism
	void setHookMotors(double val) {
		if ((topHookInductiveSensor && val > 0.0) || (bottomHookInductiveSensor && val < 0.0)) {
			hookTalon1.Set(ControlMode::PercentOutput,0);
			hookTalon2.Set(ControlMode::PercentOutput,0);
			return;
		}

		hookTalon1.Set(ControlMode::PercentOutput,val);
		hookTalon2.Set(ControlMode::PercentOutput,val);
	}

	void hookLiftMechanism() {
		if(joystickMechanisms.GetRawButton(6)) {
			setHookMotors(0.5);
		}
		else if(joystickMechanisms.GetRawButton(5)) {
			setHookMotors(-0.5);
		}
		else if(joystickMechanisms.GetRawButton(1)) {
			setHookMotors(-0.25);
		}
		else {
			setHookMotors(0.0);
		}
	}



	// CUBE MECHANISM

	void cubeMechanism() {
		if(joystickMechanisms.GetRawButton(8)) {
			setCubeArmTilt(true);
		}
		else if(joystickMechanisms.GetRawButton(7)) {
			setCubeArmTilt(false);
		}

		cubeIntakeMechanism();
		double rawAxis1 = -joystickMechanisms.GetRawAxis(1);
		if (rawAxis1 > 0.2)
		{
			if(!topInductiveSensor && cubeArmTiltSole->Get() != DoubleSolenoid::kReverse) {
				cubeLiftMotor->Set(ControlMode::PercentOutput, 1.0); // lift the cube up
			}
			else {
				cubeLiftMotor->Set(ControlMode::PercentOutput,0.0); // stop the cube lift
			}
		}
		else if (rawAxis1 < -0.2)
		{
			if(!bottomInductiveSensor && cubeArmTiltSole->Get() != DoubleSolenoid::kReverse) {
				cubeLiftMotor->Set(ControlMode::PercentOutput, -1.0); // lower the cube down
			}
			else {
				cubeLiftMotor->Set(ControlMode::PercentOutput,0.0); // stop the cube lift
			}
		}
		else
		{
			cubeLiftMotor->Set(ControlMode::PercentOutput, 0.0);
		}


	}

	void pushCubeMotorsOut()
	{
		setCubeMotors(0.25);
	}

	void pullCubeMotorsIn()
	{
		setCubeMotors(-0.8);
	}

	void stopCubeMotors()
	{
		setCubeMotors(0.0);
	}

	void setCubeMotors(double value) {
		cubeIntakeTalonLeft.Set(ControlMode::PercentOutput,value);
		cubeIntakeTalonRight.Set(ControlMode::PercentOutput,value);
	}

	void cubeIntakeMechanism() {
		double rawAxis3 = -joystickMechanisms.GetRawAxis(3);
		if(joystickMechanisms.GetRawButton(4)) { // shoot the cube out
			pushCubeMotorsOut();
		}
		else if(joystickMechanisms.GetRawButton(2)) { // take the cube in
			pullCubeMotorsIn();
		}
		else if (rawAxis3 > 0.2)
		{
			setCubeMotors(rawAxis3);
		}
		else if (rawAxis3 < -0.2)
		{
			setCubeMotors(rawAxis3);
		}

		else {
			stopCubeMotors();
		}
	}

	void setCubeArmTilt(bool value)
	{
		if (value) {
			cubeArmTiltSole->Set(DoubleSolenoid::kForward);
		}
		else {
			cubeArmTiltSole->Set(DoubleSolenoid::kReverse);
		}

	}




	// SENSORS
	void getInductiveSensors() {
		topInductiveSensor = (cubeLiftInductiveTop.GetVoltage() > 3.0 ? true : false);
		bottomInductiveSensor = (cubeLiftInductiveBottom.GetVoltage() > 3.0 ? true : false);


		topHookInductiveSensor = (hookLiftInductiveTop.GetVoltage() > 3.0 ? true: false);
		bottomHookInductiveSensor = (hookLiftInductiveBottom.GetVoltage() > 3.0 ? true : false);

	}



	void DisabledInit() {
		setRight(0.0);
		setLeft(0.0);

		rightServo->SetAngle(rightServo->GetAngle() - 90);
		leftServo->SetAngle(leftServo->GetAngle() + 90);
	}

};

START_ROBOT_CLASS(Robot)
