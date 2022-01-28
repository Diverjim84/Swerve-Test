// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Drivetrain.h"

Drivetrain::Drivetrain(constants::swerveConstants::SwerveConfig constants):
                        m_kinematics(constants.m_frontLeftLocation, 
                                      constants.m_frontRightLocation,
                                      constants.m_rearLeftLocation,
                                      constants.m_rearRightLocation),
                        m_odometry(m_kinematics, GetHeading()),
                        m_frontLeft(0, constants.frontLeftConstants),
                        m_frontRight(1, constants.frontRightConstants),
                        m_backLeft(2, constants.rearLeftConstants),
                        m_backRight(3, constants.rearRightConstants),
                        m_gyro(constants::swerveConstants::pigeonID)
{

}


void Drivetrain::Drive(units::meters_per_second_t xSpeed,
                       units::meters_per_second_t ySpeed,
                       units::radians_per_second_t rot, 
                       bool fieldRelative) 
{
  units::degree_t rawHeading{ m_gyro.GetFusedHeading()};
  
  
  auto states = m_kinematics.ToSwerveModuleStates(
      fieldRelative ? frc::ChassisSpeeds::FromFieldRelativeSpeeds(
                          xSpeed, ySpeed, rot, frc::Rotation2d(rawHeading))
                    : frc::ChassisSpeeds{xSpeed, ySpeed, rot});

  m_kinematics.DesaturateWheelSpeeds(&states, m_vt_max);

  auto [fl, fr, bl, br] = states;

  m_frontLeft.Set(fl);
  m_frontRight.Set(fr);
  m_backLeft.Set(bl);
  m_backRight.Set(br);
}

void Drivetrain::UpdateOdometry() {
  m_odometry.Update(GetHeading(), m_frontLeft.GetState(),
                    m_frontRight.GetState(), m_backLeft.GetState(),
                    m_backRight.GetState());
}
