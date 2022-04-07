// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Drivetrain.h"
#include <frc/smartdashboard/SmartDashboard.h>
#include <wpi/sendable/SendableBuilder.h>
#include <frc/MathUtil.h>


Drivetrain::Drivetrain(constants::swerveConstants::SwerveConfig constants):
                        m_frontLeft("Front Left", constants.frontLeftConstants),
                        m_frontRight("Front Right", constants.frontRightConstants),
                        m_backLeft("Back Left", constants.rearLeftConstants),
                        m_backRight("Back Right", constants.rearRightConstants),
                        m_gyro(constants::swerveConstants::PigeonID),
                        m_kinematics(constants.m_frontLeftLocation, 
                                      constants.m_frontRightLocation,
                                      constants.m_rearLeftLocation,
                                      constants.m_rearRightLocation),
                        m_odometry(m_kinematics, GetHeading())
{

  frc::SmartDashboard::PutData("Field", &m_field2d); //post Field 
  frc::SmartDashboard::PutData("Front Left Swerve Module", &m_frontLeft);
  frc::SmartDashboard::PutData("Front Right Swerve Module", &m_frontRight);
  frc::SmartDashboard::PutData("Rear Left Swerve Module", &m_backLeft);
  frc::SmartDashboard::PutData("Rear Right Swerve Module", &m_backRight);

  m_rotGain = constants::swerveConstants::RotGain;
  m_targetHeading = 0_deg;

  SetDriveInverted(constants::swerveConstants::DriveLeftInvert, constants::swerveConstants::DriveRightInvert);

}

void Drivetrain::SetDriveInverted(bool leftInvert, bool rightInvert){
  m_frontRight.SetDriveInverted(rightInvert);
  m_backRight.SetDriveInverted(rightInvert);
  m_frontLeft.SetDriveInverted(leftInvert);
  m_backLeft.SetDriveInverted(leftInvert);
}

void Drivetrain::SetTurnInverted(bool leftInvert, bool rightInvert){
  m_frontRight.SetTurnInverted(rightInvert);
  m_backRight.SetTurnInverted(rightInvert);
  m_frontLeft.SetTurnInverted(leftInvert);
  m_backLeft.SetTurnInverted(leftInvert);
}

void Drivetrain::Drive(units::meters_per_second_t xSpeed,
                       units::meters_per_second_t ySpeed,
                       units::radians_per_second_t rot, 
                       bool fieldRelative) 
{

  auto states = m_kinematics.ToSwerveModuleStates(
      fieldRelative ? frc::ChassisSpeeds::FromFieldRelativeSpeeds(
                          xSpeed, ySpeed, rot, GetHeading())
                    : frc::ChassisSpeeds{xSpeed, ySpeed, rot});

  m_kinematics.DesaturateWheelSpeeds(&states, constants::swerveConstants::MaxSpeed);

  auto [fl, fr, bl, br] = states;

  m_frontLeft.Set(fl);
  m_frontRight.Set(fr);
  m_backLeft.Set(bl);
  m_backRight.Set(br);

  m_vx_goal = xSpeed;
  m_vy_goal = ySpeed;
  m_vw_goal = rot;
}

void Drivetrain::DrivePos(units::meter_t x, units::meter_t y, units::degree_t heading){
  frc::Pose2d pose = GetPose();
  SetTargetHeading(heading);

  units::meters_per_second_t vx(constants::swerveConstants::PosGain*(x-pose.X()).value());
  units::meters_per_second_t vy(constants::swerveConstants::PosGain*(y-pose.Y()).value());
  units::radians_per_second_t w(CalcRotationSpeed(heading, pose.Rotation().Degrees()));
  Drive(vx,vy,w);

}

units::radians_per_second_t Drivetrain::CalcRotationSpeed(units::degree_t targetAngle, units::degree_t currentHeading)
{
  //frc::SmartDashboard::PutNumber("Turn Error Current Heading", currentHeading.value());
  //find difference between goal and current heading
  //frc::Rotation2d e( targetAngle-GetPose().Rotation().Degrees());
  //If angle is >180 deg, go the other way. 
  /*if(units::math::abs(e.Degrees()) > 180_deg){
    e = e + frc::Rotation2d(360_deg);
  }*/
  //1/2 degree error is ok for now. 
  frc::Rotation2d e = frc::AngleModulus(targetAngle-GetPose().Rotation().Degrees());
  return m_rotGain*(e.Degrees()/180_deg)*constants::swerveConstants::MaxAngularVelocity;

  
  
}

void Drivetrain::DriveXY(units::meters_per_second_t xSpeed, units::meters_per_second_t ySpeed){
  frc::Rotation2d heading = GetPose().Rotation();
  units::radians_per_second_t rot = CalcRotationSpeed(m_targetHeading, heading.Degrees());
  auto states = m_kinematics.ToSwerveModuleStates(frc::ChassisSpeeds::FromFieldRelativeSpeeds(xSpeed, ySpeed, rot, heading));
  
  m_kinematics.DesaturateWheelSpeeds(&states, constants::swerveConstants::MaxSpeed);

  auto [fl, fr, bl, br] = states;

  m_frontLeft.Set(fl);
  m_frontRight.Set(fr);
  m_backLeft.Set(bl);
  m_backRight.Set(br);

  m_vx_goal = xSpeed;
  m_vy_goal = ySpeed;
  m_vw_goal = rot;
}

void Drivetrain::UpdateOdometry() {

  m_odometry.Update(GetGryoHeading(), 
                    m_frontLeft.GetState(),
                    m_frontRight.GetState(), 
                    m_backLeft.GetState(),
                    m_backRight.GetState()
                  );

  m_field2d.SetRobotPose(m_odometry.GetPose());

}

frc::ChassisSpeeds Drivetrain::GetChassisSpeeds(){
  return m_kinematics.ToChassisSpeeds(m_frontLeft.GetState(),
                    m_frontRight.GetState(), 
                    m_backLeft.GetState(),
                    m_backRight.GetState());
}

void Drivetrain::SetPose(frc::Pose2d pose){
  m_odometry.ResetPosition(pose,GetGryoHeading());
  
}

frc::Rotation2d Drivetrain::GetHeading(){
  
  return m_odometry.GetPose().Rotation();
}

frc::Rotation2d Drivetrain::GetGryoHeading(){
  return frc::Rotation2d(units::degree_t(m_gyro.GetYaw()));
}

void Drivetrain::SetHeading(units::degree_t angle){
  frc::Pose2d cur_pose = GetPose();
  SetPose(frc::Pose2d(cur_pose.X(),cur_pose.Y(), frc::Rotation2d(angle)));
}

void Drivetrain::ResetDriveEncoders(){
  m_frontLeft.ResetDriveEncoder();
  m_frontRight.ResetDriveEncoder();
  m_backLeft.ResetDriveEncoder();
  m_backRight.ResetDriveEncoder();
}

void Drivetrain::InitSendable(wpi::SendableBuilder& builder){
  builder.SetSmartDashboardType("Swerve Drivetrain");

  builder.AddDoubleProperty("X Position meters", [this] { return GetPose().X().value(); }, nullptr);
  builder.AddDoubleProperty("X Position inches", [this] { return units::inch_t(GetPose().X()).value(); }, nullptr);
  builder.AddDoubleProperty("Y Position meters", [this] { return GetPose().Y().value(); }, nullptr);
  builder.AddDoubleProperty("Y Position inches", [this] { return units::inch_t(GetPose().Y()).value(); }, nullptr);
  builder.AddDoubleProperty("Heading Deg", [this] { return GetPose().Rotation().Degrees().value(); }, nullptr);
  builder.AddDoubleProperty("Raw Heading Deg", [this] { return m_gyro.GetYaw(); }, nullptr);

  builder.AddDoubleProperty("X Goal MPS", [this] { return m_vx_goal.value(); }, nullptr);
  builder.AddDoubleProperty("Y Goal MPS", [this] { return m_vy_goal.value(); }, nullptr);
  builder.AddDoubleProperty("W Goal deg/sec", [this] { return units::degrees_per_second_t(m_vw_goal).value(); }, nullptr);

  
  

}

void Drivetrain::PutChildSendables(){
  frc::SmartDashboard::PutData("Swerve Front Right", &m_frontRight);
  frc::SmartDashboard::PutData("Swerve Front Left", &m_frontLeft);
  frc::SmartDashboard::PutData("Swerve Rear Right", &m_backRight);
  frc::SmartDashboard::PutData("Swerve Rear Left", &m_backLeft);
}

void Drivetrain::SendData(){
  
  //frc::SmartDashboard::PutNumber("X Position meters", GetPose().X().value());
  frc::SmartDashboard::PutNumber("X Position inches", units::inch_t(GetPose().X()).value());
  //frc::SmartDashboard::PutNumber("Y Position meters", GetPose().Y().value());
  frc::SmartDashboard::PutNumber("Y Position inches", units::inch_t(GetPose().Y()).value());
  frc::SmartDashboard::PutNumber("Heading Deg", GetPose().Rotation().Degrees().value());
  //frc::SmartDashboard::PutNumber("Raw Heading Deg", m_gyro.GetYaw());

  frc::SmartDashboard::PutNumber("Heading Control w", m_vw_goal.value());
  frc::SmartDashboard::PutNumber("Heading Target", m_targetHeading.value());

  //frc::SmartDashboard::PutNumber("Turn Vel", m_gyro.);

  frc::SmartDashboard::PutNumber("X Goal MPS", m_vx_goal.value());
  frc::SmartDashboard::PutNumber("Y Goal MPS", m_vy_goal.value());
  frc::SmartDashboard::PutNumber("W Goal deg/sec", units::degrees_per_second_t(m_vw_goal).value());

  m_frontLeft.SendData();
  m_frontRight.SendData();
  m_backLeft.SendData();
  m_backRight.SendData();
}