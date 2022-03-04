// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/AnalogGyro.h>
#include <frc/geometry/Translation2d.h>
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/kinematics/SwerveDriveOdometry.h>
#include <wpi/numbers>
#include <frc/smartdashboard/Field2d.h>


#include "Constants.h"
#include "SwerveModule.h"

#include <ctre/Phoenix.h>

/**
 * Represents a swerve drive style drivetrain.
 */
class Drivetrain : public wpi::Sendable {
 private:
  

  SwerveModule m_frontLeft;
  SwerveModule m_frontRight;
  SwerveModule m_backLeft;
  SwerveModule m_backRight;

  units::degree_t m_targetHeading;
  double m_rotGain;

  phoenix::sensors::Pigeon2 m_gyro;

  frc::SwerveDriveKinematics<4> m_kinematics;

  frc::SwerveDriveOdometry<4> m_odometry;

  units::meters_per_second_t m_vx_goal; //X velocity target
  units::meters_per_second_t m_vy_goal; //Y velocity target
  units::radians_per_second_t m_vw_goal; //rotational velocity target

  frc::Field2d m_field2d{};

  frc::Rotation2d GetGryoHeading();

 public:
  Drivetrain(constants::swerveConstants::SwerveConfig constants);
  void InitSendable(wpi::SendableBuilder& builder);
  void PutChildSendables();
  void SendData();

  void Drive(units::meters_per_second_t xSpeed,
             units::meters_per_second_t ySpeed, 
             units::radians_per_second_t rot,
             bool fieldRelative = true);
  void DrivePos(units::meter_t x, units::meter_t y, units::degree_t heading);

  void DriveXY(units::meters_per_second_t xSpeed,
               units::meters_per_second_t ySpeed,
               units::degree_t heading){m_targetHeading = heading; DriveXY(xSpeed, ySpeed);};

  void DriveXY(units::meters_per_second_t xSpeed,
               units::meters_per_second_t ySpeed);
  
  units::radians_per_second_t CalcRotationSpeed(units::degree_t targetAngle, units::degree_t currentHeading);
  
  void SetTargetHeading(units::degree_t heading){m_targetHeading = heading;};
  units::degree_t GetTargetHeading(){return m_targetHeading;};
  
  void SetRotationGain(double gain){m_rotGain = gain;};
  double GetRotationGain(){return m_rotGain;}

  void UpdateOdometry();
  void SetPose(frc::Pose2d pose);
  frc::Pose2d GetPose(){return m_odometry.GetPose();}
  frc::ChassisSpeeds GetChassisSpeeds();

  void SetDriveInverted(bool leftInvert, bool rightInvert);
  void SetTurnInverted(bool leftInvert, bool rightInvert);

  frc::Rotation2d GetHeading();
  void SetHeading(units::degree_t angle);


 
};
