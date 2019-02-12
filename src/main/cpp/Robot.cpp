/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "Robot.h"
#include "log.h"
#include <iostream>
#include <frc/smartdashboard/SmartDashboard.h>

void Robot::RobotInit() {
  m_compressor.Start();
	m_gearbox_right.Set(frc::DoubleSolenoid::Value::kOff);
	m_gearbox_left.Set(frc::DoubleSolenoid::Value::kOff);
	m_grabber.Set(frc::DoubleSolenoid::Value::kOff);

  m_leftFollowMotor.Follow(m_leftLeadMotor);
  m_rightFollowMotor.Follow(m_rightLeadMotor);
  m_leftLeadMotor.SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);
  m_rightLeadMotor.SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);

  // set PID coefficients
  /*m_armPidController.SetP(kP);
  m_armPidController.SetI(kI);
  m_armPidController.SetD(kD);
  m_armPidController.SetIZone(kIz);
  m_armPidController.SetFF(kFF);
  m_armPidController.SetOutputRange(kMinOutput, kMaxOutput);*/

  // display PID coefficients on SmartDashboard
  frc::SmartDashboard::PutNumber("P Gain", kP);
  frc::SmartDashboard::PutNumber("I Gain", kI);
  frc::SmartDashboard::PutNumber("D Gain", kD);
  frc::SmartDashboard::PutNumber("Max Output", kMaxOutput);
  frc::SmartDashboard::PutNumber("Min Output", kMinOutput);
  frc::SmartDashboard::PutNumber("Set Rotations", 0);
}

/**
 * This function is called every robot packet, no matter the mode. Use
 * this for items like diagnostics that you want ran during disabled,
 * autonomous, teleoperated and test.
 *
 * <p> This runs after the mode specific periodic functions, but before
 * LiveWindow and SmartDashboard integrated updating.
 */
void Robot::RobotPeriodic() {}

/**
 * This autonomous (along with the chooser code above) shows how to select
 * between different autonomous modes using the dashboard. The sendable chooser
 * code works with the Java SmartDashboard. If you prefer the LabVIEW Dashboard,
 * remove all of the chooser code and uncomment the GetString line to get the
 * auto name from the text box below the Gyro.
 *
 * You can add additional auto modes by adding additional comparisons to the
 * if-else structure below with additional strings. If using the SendableChooser
 * make sure to add them to the chooser code above as wel
 * l.
 */
void Robot::AutonomousInit() {
 
}

void Robot::AutonomousPeriodic() {

}

void Robot::TeleopInit() {
  // read PID coefficients from SmartDashboard
  double p = frc::SmartDashboard::GetNumber("P Gain", 0);
  double i = frc::SmartDashboard::GetNumber("I Gain", 0);
  double d = frc::SmartDashboard::GetNumber("D Gain", 0);
  double max = frc::SmartDashboard::GetNumber("Max Output", 0);
  double min = frc::SmartDashboard::GetNumber("Min Output", 0);

  // if PID coefficients on SmartDashboard have changed, write new values to controller
  if ((p != kP)) { m_wristPidController.SetP(p); kP = p; }
  if ((i != kI)) { m_wristPidController.SetI(i); kI = i; }
  if ((d != kD)) { m_wristPidController.SetD(d); kD = d; }
  if ((max != kMaxOutput) || (min != kMinOutput)) { 
    m_wristPidController.SetOutputRange(min, max); 
    kMinOutput = min; kMaxOutput = max; 
  }
}

void Robot::TeleopPeriodic() {
  // Robot Drive
  double move   = m_stick.GetRawAxis(1);
  double rotate = m_stick.GetRawAxis(4);

  // Deadband
  if (fabs(move) < 0.15) {
    move = 0.0;
  }

  if (fabs(rotate) < 0.15) {
    rotate = 0.0;
  }

  m_robotDrive.ArcadeDrive(move, rotate);
  //m_robotDrive.ArcadeDrive(m_stick.GetRawAxis(1), m_stick.GetRawAxis(4));

  // Shifting
  if (m_stick.GetRawButton(5)) {
    LOGGER(INFO) << "High Gear";
		m_gearbox_right.Set(frc::DoubleSolenoid::Value::kForward);
    m_gearbox_left.Set(frc::DoubleSolenoid::Value::kForward);
  } else if (m_stick.GetRawButton(6)) {
    LOGGER(INFO) << "Low Gear";
		m_gearbox_right.Set(frc::DoubleSolenoid::Value::kReverse);
		m_gearbox_left.Set(frc::DoubleSolenoid::Value::kReverse);
	}

 // Grabber
 if (m_stick.GetRawButton(7)) {
    LOGGER(INFO) << "Grabber Open";
		m_grabber.Set(frc::DoubleSolenoid::Value::kForward);
  } else if (m_stick.GetRawButton(8)) {
    LOGGER(INFO) << "Grabber Close";
		m_grabber.Set(frc::DoubleSolenoid::Value::kReverse);
	} 

  // Climbing
  if (m_stick.GetRawAxis(2)) {
    LOGGER(INFO) << "Climb Arm";
    m_climbArmMotor.Set(m_stick.GetRawAxis(2));
  } else {
    m_climbArmMotor.Set(0.0);
  }

  if (m_stick.GetRawAxis(3)) {
    LOGGER(INFO) << "Climb Foot";
    m_climbFootMotor.Set(m_stick.GetRawAxis(3));
  } else {
    m_climbFootMotor.Set(0.0);
  }

  // Arm Up/Down
  if (m_stick.GetRawButton(1)) {
    LOGGER(INFO) << "Arm Down";
    m_ArmMotor.Set(0.25);
  } else if (m_stick.GetRawButton(4)) {
    LOGGER(INFO) << "Arm Up";
    m_ArmMotor.Set(-0.25);
  } else {
    m_ArmMotor.Set(0.0);
  }

  // Wrist Up/Down
  if (m_stick.GetRawButton(2)) {
    LOGGER(INFO) << "Wrist Up";
    m_wristMotor.Set(0.25);
  } else if (m_stick.GetRawButton(3)) {
    LOGGER(INFO) << "Wrist Down";
    m_wristMotor.Set(-0.25);
  } else {
    m_wristMotor.Set(0.0);
  }

  double rotations = frc::SmartDashboard::GetNumber("Set Rotations", 0);
  //m_wristPidController.SetReference(rotations, rev::ControlType::kPosition);

  LOGGER(INFO) << "  Arm Encoder: " << m_armEncoder.GetPosition();
  LOGGER(INFO) << "Wrist Encoder: " << m_wristEncoder.GetPosition();
  LOGGER(INFO) << "  C/A Encoder: " << m_climbArmEncoder.GetPosition();
  LOGGER(INFO) << "  C/F Encoder: " << m_climbFootEncoder.GetPosition();
}


void Robot::TestPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif
