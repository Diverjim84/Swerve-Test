// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once


#include <frc/controller/SimpleMotorFeedforward.h>
#include <frc/kinematics/SwerveModuleState.h>
#include <frc/geometry/Rotation2d.h>
#include <wpi/sendable/Sendable.h>
#include <units/math.h>

#include "SwerveModuleConstants.h"

#include <ctre/Phoenix.h>






class SwerveModule : public wpi::Sendable{
 public:
  SwerveModule(std::string id, SwerveModuleConstants constants);

  std::string GetID(){return m_id;};

  units::degree_t GetOffsetAngle();
  units::degree_t GetLastAngle(){return m_lastAngle;};

  void SetDriveInverted(bool invert){m_driveMotor.SetInverted(invert);};
  void SetTurnInverted(bool invert){m_turnMotor.SetInverted(invert);};

  void Set(frc::SwerveModuleState goal, bool optimize = true);
  frc::SwerveModuleState GetState();

  
  units::meters_per_second_t GetSpeedMPS();
  frc::Rotation2d GetAngle();

  void ResetDriveEncoder();
  

  void InitSendable(wpi::SendableBuilder& builder);
  void SendData();

 private:
  std::string     m_id;//module number
  units::degree_t m_angleOffset;//for zeroing the turn angle
  units::degree_t m_lastAngle;

  double m_wheelDiameter;
  double m_driveGearRatio;
  double m_turnGearRation;

  TalonFX m_driveMotor;
  TalonFX m_turnMotor;
  CANCoder m_turnEncoder;

  frc::SimpleMotorFeedforward<units::meter> m_driveFeedForward; 

   

};
