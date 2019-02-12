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

  // display PID coefficients on SmartDashboard
  frc::SmartDashboard::PutNumber("Arm P Gain",               armCoeff.kP);
  frc::SmartDashboard::PutNumber("Arm I Gain",               armCoeff.kI);
  frc::SmartDashboard::PutNumber("Arm D Gain",               armCoeff.kD);
  frc::SmartDashboard::PutNumber("Arm Max Output",           armCoeff.kMaxOutput);
  frc::SmartDashboard::PutNumber("Arm Min Output",           armCoeff.kMinOutput);

  frc::SmartDashboard::PutNumber("Wrist P Gain",             wristCoeff.kP);
  frc::SmartDashboard::PutNumber("Wrist I Gain",             wristCoeff.kI);
  frc::SmartDashboard::PutNumber("Wrist D Gain",             wristCoeff.kD);
  frc::SmartDashboard::PutNumber("Wrist Max Output",         wristCoeff.kMaxOutput);
  frc::SmartDashboard::PutNumber("Wrist Min Output",         wristCoeff.kMinOutput);

  frc::SmartDashboard::PutNumber("Climb Arm P Gain",         climbArmCoeff.kP);
  frc::SmartDashboard::PutNumber("Climb Arm I Gain",         climbArmCoeff.kI);
  frc::SmartDashboard::PutNumber("Climb Arm D Gain",         climbArmCoeff.kD);
  frc::SmartDashboard::PutNumber("Climb Arm Max Output",     climbArmCoeff.kMaxOutput);
  frc::SmartDashboard::PutNumber("Climb Arm Min Output",     climbArmCoeff.kMinOutput);

  frc::SmartDashboard::PutNumber("Climb Foot P Gain",        climbFootCoeff.kP);
  frc::SmartDashboard::PutNumber("Climb Foot I Gain",        climbFootCoeff.kI);
  frc::SmartDashboard::PutNumber("Climb Foot D Gain",        climbFootCoeff.kD);
  frc::SmartDashboard::PutNumber("Climb Foot Max Output",    climbFootCoeff.kMaxOutput);
  frc::SmartDashboard::PutNumber("Climb Foot Min Output",    climbFootCoeff.kMinOutput);

  frc::SmartDashboard::PutNumber("Arm Rotations Level 0",   armRotations[0]);
  frc::SmartDashboard::PutNumber("Arm Rotations Level 1",   armRotations[1]);
  frc::SmartDashboard::PutNumber("Arm Rotations Level 2",   armRotations[2]);
  frc::SmartDashboard::PutNumber("Arm Rotations Level 3",   armRotations[3]);
  
  frc::SmartDashboard::PutNumber("Wrist Rotations Level 0", wristRotations[0]);
  frc::SmartDashboard::PutNumber("Wrist Rotations Level 1", wristRotations[1]);
  frc::SmartDashboard::PutNumber("Wrist Rotations Level 2", wristRotations[2]);
  frc::SmartDashboard::PutNumber("Wrist Rotations Level 3", wristRotations[3]);

  frc::SmartDashboard::PutNumber("Climb Foot Rotations",    climbFootRotations);
  frc::SmartDashboard::PutNumber("Climb Arm Rotations",     climbArmRotations);
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
  LoadParameters();
}

void Robot::AutonomousPeriodic() {

}

void Robot::TeleopInit() {
  LoadParameters();
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

void Robot::LoadParameters () {
  double p, i, d, min, max;

  // read PID coefficients from SmartDashboard
  p   = frc::SmartDashboard::GetNumber("Arm P Gain", 0);
  i   = frc::SmartDashboard::GetNumber("Arm I Gain", 0);
  d   = frc::SmartDashboard::GetNumber("Arm D Gain", 0);
  min = frc::SmartDashboard::GetNumber("Arm Min Output", 0);
  max = frc::SmartDashboard::GetNumber("Arm Max Output", 0);

  // If PID coefficients on SmartDashboard have changed, write new values to controller
  if ((p != armCoeff.kP)) { m_armPidController.SetP(p); armCoeff.kP = p; }
  if ((i != armCoeff.kI)) { m_armPidController.SetI(i); armCoeff.kI = i; }
  if ((d != armCoeff.kD)) { m_armPidController.SetD(d); armCoeff.kD = d; }
  if ((max != armCoeff.kMaxOutput) || (min != armCoeff.kMinOutput)) { 
    m_armPidController.SetOutputRange(min, max); 
    armCoeff.kMinOutput = min; armCoeff.kMaxOutput = max; 
  }

  // read PID coefficients from SmartDashboard
  p   = frc::SmartDashboard::GetNumber("Wrist P Gain", 0);
  i   = frc::SmartDashboard::GetNumber("Wrist I Gain", 0);
  d   = frc::SmartDashboard::GetNumber("Wrist D Gain", 0);
  min = frc::SmartDashboard::GetNumber("Wrist Min Output", 0);
  max = frc::SmartDashboard::GetNumber("Wrist Max Output", 0);

  // If PID coefficients on SmartDashboard have changed, write new values to controller
  if ((p != wristCoeff.kP)) { m_wristPidController.SetP(p); wristCoeff.kP = p; }
  if ((i != wristCoeff.kI)) { m_wristPidController.SetI(i); wristCoeff.kI = i; }
  if ((d != wristCoeff.kD)) { m_wristPidController.SetD(d); wristCoeff.kD = d; }
  if ((max != wristCoeff.kMaxOutput) || (min != wristCoeff.kMinOutput)) { 
    m_wristPidController.SetOutputRange(min, max); 
    wristCoeff.kMinOutput = min; wristCoeff.kMaxOutput = max; 
  }

  // read PID coefficients from SmartDashboard
  p   = frc::SmartDashboard::GetNumber("Climb Foot P Gain", 0);
  i   = frc::SmartDashboard::GetNumber("Climb Foot I Gain", 0);
  d   = frc::SmartDashboard::GetNumber("Climb Foot D Gain", 0);
  min = frc::SmartDashboard::GetNumber("Climb Foot Min Output", 0);
  max = frc::SmartDashboard::GetNumber("Climb Foot Max Output", 0);

  // If PID coefficients on SmartDashboard have changed, write new values to controller
  if ((p != climbFootCoeff.kP)) { m_climbFootPidController.SetP(p); climbFootCoeff.kP = p; }
  if ((i != climbFootCoeff.kI)) { m_climbFootPidController.SetI(i); climbFootCoeff.kI = i; }
  if ((d != climbFootCoeff.kD)) { m_climbFootPidController.SetD(d); climbFootCoeff.kD = d; }
  if ((max != climbFootCoeff.kMaxOutput) || (min != climbFootCoeff.kMinOutput)) { 
    m_climbFootPidController.SetOutputRange(min, max); 
    climbFootCoeff.kMinOutput = min; climbFootCoeff.kMaxOutput = max; 
  }

  // read PID coefficients from SmartDashboard
  p   = frc::SmartDashboard::GetNumber("Climb Arm P Gain", 0);
  i   = frc::SmartDashboard::GetNumber("Climb Arm I Gain", 0);
  d   = frc::SmartDashboard::GetNumber("Climb Arm D Gain", 0);
  min = frc::SmartDashboard::GetNumber("Climb Arm Min Output", 0);
  max = frc::SmartDashboard::GetNumber("Climb Arm Max Output", 0);

  // If PID coefficients on SmartDashboard have changed, write new values to controller
  if ((p != climbArmCoeff.kP)) { m_climbArmPidController.SetP(p); climbArmCoeff.kP = p; }
  if ((i != climbArmCoeff.kI)) { m_climbArmPidController.SetI(i); climbArmCoeff.kI = i; }
  if ((d != climbArmCoeff.kD)) { m_climbArmPidController.SetD(d); climbArmCoeff.kD = d; }
  if ((max != climbArmCoeff.kMaxOutput) || (min != climbArmCoeff.kMinOutput)) { 
    m_climbArmPidController.SetOutputRange(min, max); 
    climbArmCoeff.kMinOutput = min; climbArmCoeff.kMaxOutput = max; 
  }

  armRotations[0]    = frc::SmartDashboard::GetNumber("Arm Rotations Level 0", 0);
  armRotations[1]    = frc::SmartDashboard::GetNumber("Arm Rotations Level 1", 0);
  armRotations[2]    = frc::SmartDashboard::GetNumber("Arm Rotations Level 2", 0);
  armRotations[3]    = frc::SmartDashboard::GetNumber("Arm Rotations Level 3", 0);
  wristRotations[0]  = frc::SmartDashboard::GetNumber("Wrist Rotations Level 0", 0);
  wristRotations[1]  = frc::SmartDashboard::GetNumber("Wrist Rotations Level 1", 0);
  wristRotations[2]  = frc::SmartDashboard::GetNumber("Wrist Rotations Level 2", 0);
  wristRotations[3]  = frc::SmartDashboard::GetNumber("Wrist Rotations Level 3", 0);

  climbFootRotations = frc::SmartDashboard::GetNumber("Climb Foot Rotations", 0);
  climbArmRotations  = frc::SmartDashboard::GetNumber("Climb Arm Rotations", 0);
}

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif
