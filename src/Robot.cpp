/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include <iostream>
#include <string>
#include <bits/stdc++.h>

#include <IterativeRobot.h>
#include <LiveWindow/LiveWindow.h>
#include <SmartDashboard/SendableChooser.h>
#include <SmartDashboard/SmartDashboard.h>
#include <Ultrasonic.h>
#include "ctre/Phoenix.h"
#include "WPILib.h"

#include <Timer.h>

using namespace frc;
using namespace std;






class Robot : public frc::IterativeRobot {


public:
	Timer *timer = new Timer();
	const double LIFT_CONSTANT_COEFFICIENT = 10;

	Ultrasonic *ultraLeft;
	Ultrasonic *ultraFront;
	Ultrasonic *ultraRight;
//	TalonSRX talon;
	TalonSRX talonRight1, talonRight2, talonLeft1, talonLeft2;
	TalonSRX ropeTalon1, ropeTalon2;
	// talonRight1 has encoder
	// talonLeft2 has encoder
	TalonSRX elevatorTalon; // negative pulls string in
//	AnalogInput input;
//	AnalogInput dInput;
//	AnalogInput dInput2;

	TalonSRX cubeLiftMotor;
	AnalogInput cubeLiftInductiveTop;
	AnalogInput cubeLiftInductiveBottom;

//	AnalogGyro gyro;
	ADXRS450_Gyro gyro;

	double rightEncoderConstant, leftEncoderConstant;

	TalonSRX cubeIntakeTalonLeft, cubeIntakeTalonRight;
	DoubleSolenoid *cubeArmTiltSole = new DoubleSolenoid(6 ,1); // change numbers
	DoubleSolenoid *climbArmTiltSole = new DoubleSolenoid(7,0);

	bool topInductiveSensor, bottomInductiveSensor;

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


	double v[2];

	double beginningDiff;


	Robot() : talonRight1(1), talonRight2(2),
			elevatorTalon(3),
			talonLeft1(4), talonLeft2(5),
			cubeIntakeTalonLeft(7),cubeIntakeTalonRight(8),
			joystickMain(0),
			/*dInput(1),dInput2(2),*/
			joystickMechanisms(1),
			ropeTalon1(9), ropeTalon2(10),
			cubeLiftInductiveTop(0),cubeLiftInductiveBottom(1),
			cubeLiftMotor(6)
	{

	}

	void setup()
	{
		ultraLeft = new Ultrasonic(0, 1);
		ultraLeft->SetAutomaticMode(true);
//		ultraRight = new Ultrasonic(1, 0);
//		ultraRight->SetAutomaticMode(true);
		beginningDiff = -1000000;
		moderator = 0.5;
		timer->Start();

//		rightEncoderConstant = ;
//		leftEncoderConstant = ;

//		ultraLeft->SetEnabled(true);
//		ultraRight->SetEnabled(true);
	}

	void RobotInit() {
		ultraLeft = new Ultrasonic(1, 0);
		ultraLeft->SetAutomaticMode(true);
		ultraRight = new Ultrasonic(3, 2);
		ultraRight->SetAutomaticMode(true);
		ultraFront = new Ultrasonic(5, 4);
		ultraFront->SetAutomaticMode(true);
		talonRight1.SetInverted(true);
		talonRight2.SetInverted(true);


		gyro.Calibrate();
		gyro.Reset();

		cubeIntakeTalonLeft.SetNeutralMode(NeutralMode::Brake);
		cubeIntakeTalonRight.SetNeutralMode(NeutralMode::Brake);

		cubeIntakeTalonLeft.SetInverted(true);

		cubeLiftMotor.SetNeutralMode(NeutralMode::Brake);


		talonRight1.SetNeutralMode(NeutralMode::Coast);
		talonRight2.SetNeutralMode(NeutralMode::Coast);
		talonLeft1.SetNeutralMode(NeutralMode::Coast);
		talonLeft2.SetNeutralMode(NeutralMode::Coast);

		setupEncoderTalon(&talonLeft2);
		setupEncoderTalon(&talonRight1);

		timer->Start();

//		setup();
//		ultraLeft = new Ultrasonic(0,1);
//		ultraLeft->SetAutomaticMode(true);
//		ultraLeft->SetEnabled(true);

//
//
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

	void AutonomousInit() override {

	}

	void AutonomousPeriodic() {

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

	void driveSystemNeutralMode(int a = 0)
	{
		if (a == 0)
		{
			talonRight1.SetNeutralMode(NeutralMode::Coast);
			talonRight2.SetNeutralMode(NeutralMode::Coast);
			talonLeft1.SetNeutralMode(NeutralMode::Coast);
			talonLeft2.SetNeutralMode(NeutralMode::Coast);
		}
		if (a == 1)
		{
			talonRight1.SetNeutralMode(NeutralMode::Brake);
			talonRight2.SetNeutralMode(NeutralMode::Brake);
			talonLeft1.SetNeutralMode(NeutralMode::Brake);
			talonLeft2.SetNeutralMode(NeutralMode::Brake);
		}
	}


	void TeleopInit() {
		driveSystemNeutralMode(0);

		setCubeArmTilt(false);
		climbArmTiltSole->Set(DoubleSolenoid::kReverse);


		gyro.Reset();
		timer->Start();
		timer->Reset();

		lefty  = talonLeft2.GetSelectedSensorPosition(kPIDLoopIdx);


	}

	void pushCubeMotorsOut()
	{
		setCubeMotors(1.0);
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

//	double howfar()
//	{
//		double avg =
//	}

	void goForwardInInches(double inches)
	{
		const double encoderConst = 35.34;
		const double a = 0.023944549;
		const double b = 35.24782609;
		double initRight = talonRight1.GetSelectedSensorPosition(kPIDLoopIdx);
		double initLeft  = talonLeft2.GetSelectedSensorPosition(kPIDLoopIdx);
		double rightside = 0.2;
		double leftside  = 0.2;
		setRight(rightside);
		setLeft(leftside);
		while ((inches*inches * a) + b*inches > -(initLeft - talonLeft2.GetSelectedSensorPosition(kPIDLoopIdx)))
		{
			cout << -(initLeft - talonLeft2.GetSelectedSensorPosition(kPIDLoopIdx)) << endl;
			rightside = 0.2;
			leftside = 0.2;
			double coeff = 0.004;
//			if (gyro.GetAngle() > 0)
//				rightside -= coeff * gyro.GetAngle();
//			if (gyro.GetAngle() < 0)
//				leftside  += coeff * gyro.GetAngle();
			setRight(rightside);
			setLeft(leftside);
		}
		setRight(0.0);
		setLeft(0.0);
	}

	void forwardInches()
	{
//		goForwardInInches(value - 20);
		while (ultraFront->GetRangeInches() > 21)
		{
			setRight(0.15);
			setLeft(0.15);
		}
		setRight(0.0);
		setLeft(0.0);
	}

	void TeleopPeriodic() {
//		cout << ultraFront->GetRangeMM() << "    " << gyro.GetAngle() << endl;



		getSensors();
		driveSystem();
		mechanismSystem();
//		cout << ultraLeft->GetRangeInches() << "     " << ultraRight->GetRangeInches() << endl;
		if (ultraLeft->GetRangeInches() > 20 && ultraRight->GetRangeInches() > 20)
		{
			cout << "YES" << endl;
		}
		else
		{
			cout << "NO" << endl;
		}

//		if (joystickMain.GetRawButton(2))
//		{
//			forwardInches(120);
//			setCubeArmTilt(true);
//			timer->Reset();
//			while (timer->Get() < 2)
//			{
//				cout << timer->Get() << endl;
//
//			}
//			pushCubeMotorsOut();
//		}

//		if (joystickMain.GetRawButton(2))
//		{
//			goForwardInInches(140);
//			turn(90);
//			forwardInches();
//			timer->Reset();
//			while (timer->Get() < 2)
//			{
//				cout << timer->Get() << endl;
//			}
//			pushCubeMotorsOut();
//		}




//		cout << lefty - talonLeft2.GetSelectedSensorPosition(kPIDLoopIdx) << " " << ultraFront->GetRangeInches()<< endl;

	}


	void turn(double angle)
	{
		gyro.Reset();
		if (angle > 0)
		{
			double begDiff = angle;
			while (gyro.GetAngle() < angle)
			{
				double speed = 0.1 + 0.1*(1 -  gyro.GetAngle()/angle);
				setRight(-speed);
				setLeft(speed);
			}
		}
		else if (angle < 0)
		{
			while (gyro.GetAngle() > angle)
			{
				setRight(0.16);
				setLeft(-0.16);
			}
		}
		stopDriveMotors();
	}

	void stopDriveMotors()
	{
		setRight(0.0);
		setLeft(0.0);
	}


	void setRopeTalon(double value)
	{
		ropeTalon1.Set(ControlMode::PercentOutput, value);
		ropeTalon2.Set(ControlMode::PercentOutput, value);
	}


	void DisabledInit() {
		talonRight1.Set(ControlMode::PercentOutput, 0.0);
		talonLeft1.Set(ControlMode::PercentOutput, 0.0);
	}

	double convertAngle(double val)
	{
		return (double)(val - initialValTalon)/2000*360;
	}

	double getLiftPosition()
	{
		return cubeLiftMotor.GetSelectedSensorPosition(kPIDLoopIdx) - initialLiftPosition;
	}

	double getLiftPositionInCentimeters()
	{
		return (getLiftPosition()/LIFT_CONSTANT_COEFFICIENT);
	}

	/*
	void setRotalon(double val)

	{
		cout << input.GetVoltage() << endl;
		if (input.GetVoltage() > 3.0 && val < 0)
		{
			talon.Set(ControlMode::PercentOutput, 0.0);
			return;
		}
		talon.Set(ControlMode::PercentOutput, val);
	}

	void turn(double angle)
	{
		if (beginningDiff == -1000000)
			beginningDiff = convertAngle(talon.GetSelectedSensorPosition(kPIDLoopIdx)) - angle;
		double talonAngle = convertAngle((talon.GetSelectedSensorPosition(kPIDLoopIdx)));
		if (angle > 0)
		{
			if (talonAngle <= angle)
			{
				double speed = (abs(convertAngle(talon.GetSelectedSensorPosition(kPIDLoopIdx)) - angle)/beginningDiff)*0.40 + 0.04;
				setRotalon(-speed);
			}
			else
			{
				setRotalon(0);
				beginningDiff = -1000000;
			}
		}
		if (angle < 0)
		{
			if (talonAngle >= angle)
			{
				double speed = (abs(convertAngle(talon.GetSelectedSensorPosition(kPIDLoopIdx)) - angle)/beginningDiff)*0.40 + 0.04;
				setRotalon(speed);
			}
			else
			{
				setRotalon(0);
				beginningDiff = -1000000;
			}
		}
	}

*/

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


	void driveSystem()
	{
		if (joystickMain.GetRawButton(2)) // if green a button is pressed
			moderator = 0.85; // makes robot go faster
		else if (joystickMain.GetRawButton(3)) // if red b button is pressed
			moderator = 0.2; // make it really slow
		else // base case let it be half speed
			moderator = 0.5; // limits the range given from the controller

		j_x = joystickMain.GetRawAxis(1) * moderator;
		j_y = joystickMain.GetRawAxis(0) * moderator;

		double speedL = +j_y - j_x;
		double speedR = -j_y - j_x;


		setLeft(speedL);
		setRight(speedR);


	}


	void getSensors() {
		topInductiveSensor = (cubeLiftInductiveTop.GetVoltage() > 3.0 ? true : false);
		bottomInductiveSensor = (cubeLiftInductiveBottom.GetVoltage() > 3.0 ? true : false);
	}

	void mechanismSystem()
	{
		cubeMechanism();

		if (joystickMechanisms.GetRawButton(5))
		{
			elevatorTalon.Set(ControlMode::PercentOutput, 1.0);
		}
		else if (joystickMechanisms.GetRawButton(6))
		{
			elevatorTalon.Set(ControlMode::PercentOutput, -1.0);
		}
		else
		{
			elevatorTalon.Set(ControlMode::PercentOutput, 0.0);
		}



		// climb system

		if(joystickMechanisms.GetRawButton(9)) {
			climbArmTiltSole->Set(DoubleSolenoid::kForward);
		}
		else if(joystickMechanisms.GetRawButton(10)) {
			climbArmTiltSole->Set(DoubleSolenoid::kReverse);
		}

//		if(joystickMain.GetRawButton(6)) {
//			//up
//			setRopeMotors(0.2);
//		}
//		else if(joystickMain.GetRawButton(5)) {
//			setRopeMotors(-0.2);
//		}
//		else {
//			setRopeMotors(0.0);
//		}


		//////climbbbbbbb
//		double mag = joystickMechanisms.GetMagnitude();
//		if ( mag > 0.3) {
//			setRopeMotors(-joystickMechanisms.GetRawAxis(1)/2);
//		}
//		else {
//			setRopeMotors(0.0);
//		}

	}

	void setRopeMotors(double val) {
		ropeTalon1.Set(ControlMode::PercentOutput,val);
		ropeTalon2.Set(ControlMode::PercentOutput,val);
	}

	void cubeMechanism() {
		if(joystickMechanisms.GetRawButton(8)) {
			setCubeArmTilt(true);
		}
		else if(joystickMechanisms.GetRawButton(7)) {
			setCubeArmTilt(false);
		}

		cubeIntakeMechanism();

		if(joystickMechanisms.GetRawButton(4)) {
			cubeLiftMechanism(1); // lift up
		}
		else if(joystickMechanisms.GetRawButton(2)) {
			cubeLiftMechanism(-1); // lift down
		}
		else {
			cubeLiftMechanism(); // stop the lift
		}


	}

	void cubeLiftMechanism(int pos = -10) {
		// if (getLiftPositionInCentimeters() < 0)
			// cubeLiftMotor.Set(ControlMode::PercentOutput, 0.1);
		getSensors();
//		cout << bottomInductiveSensor << endl;


		if(pos == 1) {
			if(!topInductiveSensor) {
				cubeLiftMotor.Set(ControlMode::PercentOutput,0.75); // lift the cube up
			}
			else {
				cubeLiftMotor.Set(ControlMode::PercentOutput,0.0); // stop the cube lift
			}
		}
		else if (pos == -1) {
			if(!bottomInductiveSensor) {
				cubeLiftMotor.Set(ControlMode::PercentOutput,-0.50); // lower the cube down
			}
			else {
				cubeLiftMotor.Set(ControlMode::PercentOutput,0.0); // stop the cube lift
			}
		}
		else if (pos == 4) //
		{
			setCubeLiftMotorBetweenPositions(25, 20);
		}
		else if (pos == 5) // bring lift to up position
		{
			setCubeLiftMotorBetweenPositions(10, 5);
		}
		else {
			cubeLiftMotor.Set(ControlMode::PercentOutput,0.0); // stop the cube lift
		}

	}

	void setCubeLiftMotorBetweenPositions(double pos1, double pos2)
	{
		double liftPosition = getLiftPositionInCentimeters();
		if (liftPosition <= pos1 && liftPosition >= pos2)
		{
			cubeLiftMechanism(-10);
		}
		else if (liftPosition > pos1)
		{
			cubeLiftMechanism(1);
		}
		else if (liftPosition < pos2)
		{
			cubeLiftMechanism(-1);
		}
	}

	void cubeIntakeMechanism() {
		if(joystickMechanisms.GetRawButton(1)) { // shoot the cube out
			pushCubeMotorsOut();
		}
		else if(joystickMechanisms.GetRawButton(3)) { // take the cube in
			pullCubeMotorsIn();
		}
		else { // stop the intake motors
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

	void TestPeriodic() {
//		if (joystickMain.GetRawButton(2))
//		{
//			setRotalon(-0.1);
////			cout << "h" << endl;
//		}
//		else if (joystickMain.GetRawButton(1))
//		{
////			cout << "hi" << endl;
//			setRotalon(0.1);
//		}
//		else
//		{
//			cout << endl;
//			setRotalon(0);
//		}
//	}
	}

};

START_ROBOT_CLASS(Robot)
