
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

	Ultrasonic *ultraFront;
	TalonSRX talonRight1, talonRight2, talonLeft1, talonLeft2;
	TalonSRX ropeTalon1, ropeTalon2;
	TalonSRX elevatorTalon;

	TalonSRX cubeLiftMotor;
	AnalogInput cubeLiftInductiveTop;
	AnalogInput cubeLiftInductiveBottom;

	ADXRS450_Gyro gyro;

	double rightEncoderConstant, leftEncoderConstant;

	TalonSRX cubeIntakeTalonLeft, cubeIntakeTalonRight;
	DoubleSolenoid *cubeArmTiltSole = new DoubleSolenoid(6 ,1);
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

	double beginningDiff;

	string gameData;
	Preferences* preferences;

	bool done = false;



	// SETUP SECTION
	//
	//
	//

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
		preferences = Preferences::GetInstance();
	}

	void setup()
	{
		beginningDiff = -1000000;
		moderator = 0.5;
		timer->Start();

	}

	void RobotInit() {
		CameraServer::GetInstance()->StartAutomaticCapture();

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
			driveSystemCoastMode(false);
		}



	void TestPeriodic() {
		gyro.Reset();
		setCubeArmTilt(true);

		driveSystemCoastMode(false);

		goForwardInInches(36);
		turn(90);
		goForwardInInches(36);
		turn(-90);
		forwardUltrasonic();

		shootCubeOutAuto();


	}

	void AutonomousInit() {
		gyro.Reset();
		climbArmTiltSole->Set(DoubleSolenoid::kReverse);

		gameData = DriverStation::GetInstance().GetGameSpecificMessage();
		setCubeArmTilt(false);

		driveSystemCoastMode(false);
		done= false;
	}


	void AutonomousPeriodic() {
		if(gameData.length() > 0 && !done)
		{
			done = true;
			int pos = preferences->GetInt("autoPos", 0); // 1 left 2 center 3 right
			int dropPos = preferences->GetInt("dropPos", 1); // 0 - A, 1 - B
			char switchPos = gameData[0];

			climbArmTiltSole->Set(DoubleSolenoid::kForward);

			if(pos == 1) {
				if(switchPos == 'L') {
					if (dropPos == 1) {
						goForwardInInches(168-28);
						turn(90);
						forwardUltrasonic();
						shootCubeOutAuto();
					}
					else if(dropPos == 0) {
						goForwardInInches(98-28);
						turn(90);
						goForwardInInches(36);
						turn(-90);
						forwardUltrasonic();
						shootCubeOutAuto();
					}
				}
				else if (switchPos == 'R') {
					goForwardInInches(49-28);
					turn(90);
					goForwardInInches(160);
					turn(-90);
					forwardUltrasonic();
					shootCubeOutAuto();
				}
			}
			else if(pos == 2) { // center
				if(switchPos == 'L') {
					goForwardInInches(49-28);
					turn(-90);
					goForwardInInches(84);
					turn(90);
					forwardUltrasonic();
					shootCubeOutAuto();
				}
				else if (switchPos == 'R') {
					goForwardInInches(49-28);
					turn(90);
					goForwardInInches(36);
					turn(-90);
					forwardUltrasonic();
					shootCubeOutAuto();
				}
			}
			else if(pos == 3) { // right
				if(switchPos == 'L') {
					goForwardInInches(49-28);
					turn(-90);
					goForwardInInches(160);
					turn(90);
					forwardUltrasonic();
					shootCubeOutAuto();
				}
				else if (switchPos == 'R') {
					if(dropPos == 1) {
						goForwardInInches(168-28);
						turn(-90);
						forwardUltrasonic();
						shootCubeOutAuto();
					}
					else if(dropPos == 0) {
						goForwardInInches(98-28);
						turn(-90);
						goForwardInInches(36);
						turn(90);
						forwardUltrasonic();
						shootCubeOutAuto();

					}
				}
			}

			setRight(0.0);
			setLeft(0.0);
			setCubeMotors(0.0);
		}
	}

	void shootCubeOutAuto() {
		setCubeMotors(0.2);
		double start = timer->Get();
		while(timer->Get() - start < 1) {
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

	void goForwardInInches(double inches)
	{

		const double encoderConst = 35.34; // Ratio adjustment from encoder ticks to inches
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
			//0.2 before
			rightside = 0.4;
			leftside = 0.4;
			double coeff = 0.004;
			setRight(rightside);
			setLeft(leftside);
		}
		setRight(0.0);
		setLeft(0.0);
	}

	void forwardUltrasonic()
	{
		double initDist = ultraFront->GetRangeInches();
		int counter = 0;
		while (ultraFront->GetRangeInches() > 21)
		{
			if (counter < 20)
			{
				initDist = initDist * counter + ultraFront->GetRangeInches();
				counter++;
				initDist /= counter;
			}
			double ultraVal = ultraFront->GetRangeInches();
			double speed = 0.15 + 0.15 * ((ultraVal > initDist ? initDist : ultraVal) / initDist);
			setRight(speed);
			setLeft(speed);
		}
		setRight(0.0);
		setLeft(0.0);
	}

	double convertAngle(double val)
	{
		return (double)(val - initialValTalon)/2000*360;
	}

	void turn(double angle,double initial=0.25)
		{
			gyro.Reset();
			if (angle > 0)
			{
				while (gyro.GetAngle() < angle)
				{
					double speed = initial + initial * (1 -  gyro.GetAngle() / angle);
					setRight(-speed);
					setLeft(speed);
				}
			}
			else if (angle < 0)
			{
				while (gyro.GetAngle() > angle)
				{
					double speed = initial + initial * (1 - abs(gyro.GetAngle() / angle));
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

		lefty  = talonLeft2.GetSelectedSensorPosition(kPIDLoopIdx);
	}

	void TeleopPeriodic() {
		getInductiveSensors();      //uncomment to drive
		driveSystem();      //uncomment to drive
		mechanismSystem(); //uncomment to drive
	}






	// DRIVE SYSTEM

	void driveSystem()
	{
		if (joystickMain.GetRawButton(2)) // if green a button is pressed
			moderator =0.85; // makes robot go faster .. 1.0 for carpet
		else if (joystickMain.GetRawButton(3)) // if red b button is pressed
			moderator = 0.2; // make it really slow
		else // base case let it be half speed
			moderator = 0.5; // limits the range given from the controller // 0.85 for carpet

		j_x = joystickMain.GetRawAxis(1) * moderator;
		j_y = joystickMain.GetRawAxis(0) * moderator;

		double speedL = +j_y - j_x;
		double speedR = -j_y - j_x;


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
		climbMechanism();
	}



	// Rope Mechanism

	void setRopeMotors(double val) {
		ropeTalon1.Set(ControlMode::PercentOutput,val);
		ropeTalon2.Set(ControlMode::PercentOutput,val);
	}

	void climbArmTiltMechanism() {

		if(joystickMechanisms.GetRawButton(9)) {
			climbArmTiltSole->Set(DoubleSolenoid::kForward);
		}
		else if(joystickMechanisms.GetRawButton(10)) {
			climbArmTiltSole->Set(DoubleSolenoid::kReverse);
		}
	}

	void controlRopeMotors() {
		double mag = joystickMain.GetRawAxis(3);
		if ( mag > 0.3 && joystickMain.GetRawButton(1)) {
			setRopeMotors(mag/2);
		}
		else if(joystickMain.GetRawButton(1) && joystickMain.GetRawButton(4)) {
			setRopeMotors(-0.1);
		}
		else {
			setRopeMotors(0.0);
		}

	}

	void climbMechanism() {
		climbArmTiltMechanism();
		controlRopeMotors();
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
			if(!topInductiveSensor) {
				cubeLiftMotor.Set(ControlMode::PercentOutput, rawAxis1); // lift the cube up
			}
			else {
				cubeLiftMotor.Set(ControlMode::PercentOutput,0.0); // stop the cube lift
			}
		}
		else if (rawAxis1 < -0.2)
		{
			if(!bottomInductiveSensor) {
				cubeLiftMotor.Set(ControlMode::PercentOutput, rawAxis1); // lower the cube down
			}
			else {
				cubeLiftMotor.Set(ControlMode::PercentOutput,0.0); // stop the cube lift
			}
		}
		else
		{
			cubeLiftMotor.Set(ControlMode::PercentOutput, 0.0);
		}

		cubeLiftMechanism();
	}

	void cubeLiftMechanism() {
		if (joystickMain.GetRawButton(5))
		{
			elevatorTalon.Set(ControlMode::PercentOutput, 1.0);
		}
		else if (joystickMain.GetRawButton(6))
		{
			elevatorTalon.Set(ControlMode::PercentOutput, -1.0);
		}
		else
		{
			elevatorTalon.Set(ControlMode::PercentOutput, 0.0);
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
		else if (joystickMechanisms.GetRawButton(6))
		{
			setCubeMotors(0.35);
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
	}



	void DisabledInit() {
		setRight(0.0);
		setLeft(0.0);
	}

};

START_ROBOT_CLASS(Robot)
