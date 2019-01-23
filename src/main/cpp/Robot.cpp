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
  m_chooser.SetDefaultOption(kAutoNameDefault, kAutoNameDefault);
  m_chooser.AddOption(kAutoNameCustom, kAutoNameCustom);
  frc::SmartDashboard::PutData("Auto Modes", &m_chooser);

  m_compressor.Start();
	m_gearbox_right.Set(frc::DoubleSolenoid::Value::kOff);
	m_gearbox_left.Set(frc::DoubleSolenoid::Value::kOff);
	m_grabber.Set(frc::DoubleSolenoid::Value::kOff);

  m_leftFollowMotor.Follow(m_leftLeadMotor);
  m_rightFollowMotor.Follow(m_rightLeadMotor);
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
  m_autoSelected = m_chooser.GetSelected();
  // m_autoSelected = SmartDashboard::GetString(
  //     "Auto Selector", kAutoNameDefault);
  std::cout << "Auto selected: " << m_autoSelected << std::endl;

  if (m_autoSelected == kAutoNameCustom) {
    // Custom Auto goes here
  } else {
    // Default Auto goes here
  }
}

void Robot::AutonomousPeriodic() {
  if (m_autoSelected == kAutoNameCustom) {
    // Custom Auto goes here
  } else {
    // Default Auto goes here
  }
}

void Robot::TeleopInit() {}

void Robot::TeleopPeriodic() {
  // Robot drive
  m_robotDrive.ArcadeDrive(m_stick.GetRawAxis(4), -m_stick.GetRawAxis(1));

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
}


void Robot::TestPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif
