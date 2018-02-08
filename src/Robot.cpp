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



#define LIFT_CONSTANT_COEFFICIENT 10


class Robot : public frc::IterativeRobot {
	
public:
	Timer *timer = new Timer();

	Ultrasonic *ultra;
	TalonSRX talon;
	TalonSRX talonRight1, talonRight2, talonLeft1, talonLeft2;
	AnalogInput input;
	AnalogInput dInput;
	AnalogInput dInput2;

	TalonSRX cubeLiftMotor;
	AnalogInput cubeLiftInductiveTop;
	AnalogInput cubeLiftInductiveBottom;


	TalonSRX cubeIntakeTalon1, cubeIntakeTalon2;
	DoubleSolenoid *cubeArmTilt1Sole = new DoubleSolenoid(7 ,6); // change numbers
	DoubleSolenoid *cubeArmTilt2Sole = new DoubleSolenoid(8 ,9); // change numbers

	bool topInductiveSensor, bottomInductiveSensor;

	bool initialValueSet = false;
	double initialLiftPosition = 0;

	int kTimeoutMs = 10;
	int kPIDLoopIdx = 0;
	int kSlotIdx = 0;

	double initialValTalon;
	Joystick joystickMain;
	Joystick joystickMechanisms;

	double j_x, j_y;
	double moderator;


	double beginningDiff;


	Robot() : talonRight1(1), talonRight2(2),
			talonLeft1(3), talonLeft2(4),
			cubeIntakeTalon1(5),cubeIntakeTalon2(6),
			talon(6), joystickMain(0),
			input(0),dInput(1),dInput2(2),
			joystickMechanisms(1),
			cubeLiftInductiveTop(0),cubeLiftInductiveBottom(1),
			cubeLiftMotor(7)
	{

	}

	void RobotInit() {
		ultra = new Ultrasonic(8,9);
		beginningDiff = -1000000;
		int absPos = talon.GetSelectedSensorPosition(0) & 0xFFF;
//		talon.SetSelectedSensorPosition(absPos, kPIDLoopIdx, kTimeoutMs);

		moderator = 0.5;
		timer->Start();
//
//		talon.ConfigSelectedFeedbackSensor(FeedbackDevice::CTRE_MagEncoder_Relative, kPIDLoopIdx, kTimeoutMs);
//		talon.SetSensorPhase(true);
//
//		talon.ConfigNominalOutputForward(0, kTimeoutMs);
//		talon.ConfigNominalOutputReverse(0, kTimeoutMs);
//		talon.ConfigPeakOutputForward(1, kTimeoutMs);
//		talon.ConfigPeakOutputReverse(-1, kTimeoutMs);
//
//		talon.Config_kF(kPIDLoopIdx, 0.0, kTimeoutMs);
//		talon.Config_kP(kPIDLoopIdx, 0.1, kTimeoutMs);
//		talon.Config_kI(kPIDLoopIdx, 0.0, kTimeoutMs);
//		talon.Config_kD(kPIDLoopIdx, 0.0, kTimeoutMs);
	}

	void AutonomousInit() override {
		timer->Reset();
	}


	double getLiftPosition()
	{
		return cubeLiftMotor.GetSelectedSensorPosition(kPIDLoopIdx) - initialLiftPosition;
	}

	double getLiftPositionInCentimeters()
	{
		return getLiftPosition/LIFT_CONSTANT_COEFFICIENT;
	}

	void AutonomousPeriodic() {
		getSensors();

		if(timer->Get() < 1.0) {
			cubeLiftMechanism(0);
		}
		else {
			if(!initialValueSet) {
				cubeLiftMechanism(1);

				if(bottomInductiveSensor) {
					initialValueSet = true;
					initialLiftPosition = cubeLiftMotor.GetSelectedSensorPosition(kPIDLoopIdx);
				}
			}
			else {
				cubeLiftMechanism(-10);
			}
		}



	}

	void TeleopInit() {
		initialValTalon = talon.GetSelectedSensorPosition(kPIDLoopIdx);
		ultra->SetAutomaticMode(true);
		ultra->SetEnabled(true);

		timer->Reset();

	}

	double convertAngle(double val)
	{
		return (double)(val - initialValTalon)/2000*360;
	}

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

		double speedL = -j_y + j_x;
		double speedR = +j_y + j_x;


		setLeft(speedL);
		setRight(speedR);


	}

	void TeleopPeriodic() {

//		cout << ultra->GetRangeMM()/10.0 << endl;
//		cout << talon.GetSelectedSensorPosition(kPIDLoopIdx) - initialValTalon << endl;
//		if (joystickMain.GetRawButton(1))
//			talon.Set(ControlMode::PercentOutput, -0.05);
//		else if (joystickMain.GetRawButton(2))
//			talon.Set(ControlMode::PercentOutput, 0.05);
//		else
//			talon.Set(ControlMode::PercentOutput, 0);
//		turn(-360.0);
//		cout << input.GetVoltage() << "     ";
//		cout << input.GetValue() << "      ";
//		cout << dInput.GetVoltage() << "    ";
//		cout << dInput2.GetVoltage() << "    ";
//
//		cout << endl;


//		if (joystickMain.GetRawButton(2))
//		{
//			setRotalon(-0.7);
////			cout << "h" << endl;
//		}
//		else if (joystickMain.GetRawButton(1))
//		{
////			cout << "hi" << endl;
//			setRotalon(+0.7);
//		}
//		else
//		{
//			setRotalon(0);
//		}

		getSensors();
		driveSystem();
		mechanismSystem();
	}

	void getSensors() {
		topInductiveSensor = (cubeLiftInductiveTop.GetVoltage() > 3.0 ? true : false);
		bottomInductiveSensor = (cubeLiftInductiveBottom.GetVoltage() > 3.0 ? true : false);
	}

	void mechanismSystem()
	{
		cubeMechanism();
	}

	void cubeMechanism() {
		if(joystickMechanisms.GetRawButton(2)) {
			setCubeArmTilt(false);
		}
		else if(joystickMechanisms.GetRawButton(4)) {
			setCubeArmTilt(true);
		}

		cubeIntakeMechanism();

		if(joystickMechanisms.GetRawButton(3)) {
			cubeLiftMechanism(0); // lift up
		}
		else if(joystickMechanisms.GetRawButton(1)) {
			cubeLiftMechanism(1); // lift down
		}
		else {
			cubeLiftMechanism(-10); // stop the lift
		}


	}

	void cubeLiftMechanism(int pos) {
		if(pos == 0) {
			if(!topInductiveSensor) {
				cubeLiftMotor.Set(ControlMode::PercentOutput,0.3); // lift the cube up
			}
		}
		else if (pos == 1) {
			if(!bottomInductiveSensor) {
				cubeLiftMotor.Set(ControlMode::PercentOutput,-0.3); // lower the cube down
			}
		}
		else if (pos == -10){
			cubeLiftMotor.Set(ControlMode::PercentOutput,0.0); // stop the cube lift
		}

		else if (pos == 4) //
		{
			setCubeLiftMotorBetweenPositions(25, 20);
		}
		else if (pos == 5) // bring lift to up position
		{
			setCubeLiftMotorBetweenPositions(10, 5);
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
			cubeLiftMechanism(0);
		}
	}

	void cubeIntakeMechanism() {
		if(joystickMechanisms.GetRawButton(6)) { // shoot the cube out
			cubeIntakeTalon1.Set(ControlMode::PercentOutput,0.5);
			cubeIntakeTalon2.Set(ControlMode::PercentOutput,0.5);
		}
		else if(joystickMechanisms.GetRawButton(5)) { // take the cube in
			cubeIntakeTalon1.Set(ControlMode::PercentOutput,-0.5);
			cubeIntakeTalon2.Set(ControlMode::PercentOutput,-0.5);
		}
		else { // stop the intake motors
			cubeIntakeTalon1.Set(ControlMode::PercentOutput,0.0);
			cubeIntakeTalon2.Set(ControlMode::PercentOutput,0.0);
		}
	}

	void setCubeArmTilt(bool value)
	{
		if ( value) {
			cubeArmTilt1Sole->Set(DoubleSolenoid::kForward);
			cubeArmTilt2Sole->Set(DoubleSolenoid::kForward);
		}
		else {
			cubeArmTilt1Sole->Set(DoubleSolenoid::kReverse);
			cubeArmTilt2Sole->Set(DoubleSolenoid::kReverse);
		}
	}

	void TestPeriodic() {
		if (joystickMain.GetRawButton(2))
		{
			setRotalon(-0.1);
//			cout << "h" << endl;
		}
		else if (joystickMain.GetRawButton(1))
		{
//			cout << "hi" << endl;
			setRotalon(0.1);
		}
		else
		{
			cout << endl;
			setRotalon(0);
		}
	}
};

START_ROBOT_CLASS(Robot)
