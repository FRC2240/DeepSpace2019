/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <string>

#include <frc/TimedRobot.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <frc/Compressor.h>
#include <frc/DoubleSolenoid.h>
#include <frc/Joystick.h>
#include <frc/drive/DifferentialDrive.h>
#include "rev/CANSparkMax.h"


class Robot : public frc::TimedRobot {
 public:
  void RobotInit() override;
  void RobotPeriodic() override;
  void AutonomousInit() override;
  void AutonomousPeriodic() override;
  void TeleopInit() override;
  void TeleopPeriodic() override;
  void TestPeriodic() override;

 private:
  static const int leftLeadDeviceID = 3, rightLeadDeviceID = 7, leftFollowDeviceID = 4, rightFollowDeviceID = 8;
  static const int armDeviceID = 1, wristDeviceID = 2, climbArmDeviceID = 5, climbFootDeviceID = 6;

  frc::SendableChooser<std::string> m_chooser;
  const std::string kAutoNameDefault = "Default";
  const std::string kAutoNameCustom = "My Auto";
  std::string m_autoSelected;

  // Pneumatics
  frc::DoubleSolenoid m_gearbox_right{0, 1};
  frc::DoubleSolenoid m_gearbox_left{2, 3};
  frc::DoubleSolenoid m_grabber{4, 5};

  frc::Compressor m_compressor;

  // Drive Motors
  rev::CANSparkMax m_leftLeadMotor{leftLeadDeviceID, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax m_rightLeadMotor{rightLeadDeviceID, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax m_leftFollowMotor{leftFollowDeviceID, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax m_rightFollowMotor{rightFollowDeviceID, rev::CANSparkMax::MotorType::kBrushless};

  // Arm and Wrist Motors
  rev::CANSparkMax m_ArmMotor{leftLeadDeviceID, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax m_wristMotor{wristDeviceID, rev::CANSparkMax::MotorType::kBrushless};

  // Climber Motors
  rev::CANSparkMax m_climbArmMotor{climbArmDeviceID, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax m_climbFootMotor{climbFootDeviceID, rev::CANSparkMax::MotorType::kBrushless};
 
  // Robot Drive
  frc::DifferentialDrive m_robotDrive{m_leftLeadMotor, m_rightLeadMotor};

  // Joystick
  frc::Joystick m_stick{0};
};
