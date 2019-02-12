/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <string>

#include <frc/TimedRobot.h>
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
  void LoadParameters();

 private:
  static const int leftLeadDeviceID = 7, rightLeadDeviceID = 3, leftFollowDeviceID = 8, rightFollowDeviceID = 4;
  static const int armDeviceID = 1, wristDeviceID = 2, climbArmDeviceID = 5, climbFootDeviceID = 6;

  // Pneumatics
  frc::DoubleSolenoid m_gearbox_right{3, 4};
  frc::DoubleSolenoid m_gearbox_left{2, 5};
  frc::DoubleSolenoid m_grabber{1, 6};

  frc::Compressor m_compressor;

  // Drive Motors
  rev::CANSparkMax m_leftLeadMotor{leftLeadDeviceID, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax m_rightLeadMotor{rightLeadDeviceID, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax m_leftFollowMotor{leftFollowDeviceID, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax m_rightFollowMotor{rightFollowDeviceID, rev::CANSparkMax::MotorType::kBrushless};

  // Arm and Wrist Motors
  rev::CANSparkMax m_ArmMotor{armDeviceID, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax m_wristMotor{wristDeviceID, rev::CANSparkMax::MotorType::kBrushless};

  // Climber Motors
  rev::CANSparkMax m_climbArmMotor{climbArmDeviceID, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax m_climbFootMotor{climbFootDeviceID, rev::CANSparkMax::MotorType::kBrushless};
 
  // Robot Drive
  frc::DifferentialDrive m_robotDrive{m_leftLeadMotor, m_rightLeadMotor};

  // Joystick
  frc::Joystick m_stick{0};

  // Encoders
  rev::CANEncoder m_armEncoder       = m_ArmMotor.GetEncoder();
  rev::CANEncoder m_wristEncoder     = m_wristMotor.GetEncoder();
  rev::CANEncoder m_climbArmEncoder  = m_climbArmMotor.GetEncoder();
  rev::CANEncoder m_climbFootEncoder = m_climbFootMotor.GetEncoder();

  // PID Controllers
   rev::CANPIDController m_armPidController       = m_ArmMotor.GetPIDController();
   rev::CANPIDController m_wristPidController     = m_wristMotor.GetPIDController();
   rev::CANPIDController m_climbArmPidController  = m_climbArmMotor.GetPIDController();
   rev::CANPIDController m_climbFootPidController = m_climbFootMotor.GetPIDController();

  // PID coefficients
  struct pidCoeff {
    double kP;
    double kI;
    double kD;
    double kIz;
    double kFF;
    double kMinOutput;
    double kMaxOutput;
  };

  pidCoeff       armCoeff {0.1, 0.0, 1.0, 0.0, 0.0, -1.0, 1.0};
  pidCoeff     wristCoeff {0.1, 0.0, 1.0, 0.0, 0.0, -1.0, 1.0};
  pidCoeff climbFootCoeff {0.1, 0.0, 1.0, 0.0, 0.0, -1.0, 1.0};
  pidCoeff  climbArmCoeff {0.1, 0.0, 1.0, 0.0, 0.0, -1.0, 1.0};

  // Set Points for arm/wrist positions
  double armRotations[4]    {0.0, 0.0, 0.0, 0.0};
  double wristRotations[4]  {0.0, 0.0, 0.0, 0.0};
  double climbFootRotations = 0.0;
  double climbArmRotations  = 0.0;
};
