// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "SwerveModule.h"

#include <frc/geometry/Rotation2d.h>
#include <wpi/numbers>
#include "Constants.h"
#include "CTREHelpers.h"
#include <frc/smartdashboard/SmartDashboard.h>
#include <wpi/sendable/SendableBuilder.h>
#include <string.h>


SwerveModule::SwerveModule(std::string id, SwerveModuleConstants constants)
    : m_driveMotor(constants.m_driveMotorID),
      m_turnMotor(constants.m_turnMotorID),
      m_turnEncoder(constants.m_encoderID),
      m_driveFeedForward() 
{

    m_id = id;
    
    //reset devices
    m_driveMotor.ConfigFactoryDefault();
    m_turnMotor.ConfigFactoryDefault();
    m_turnEncoder.ConfigFactoryDefault();
    //Load all the config for motors and encoder
    m_driveMotor.ConfigAllSettings(constants.m_driveMotorConfig);
    m_turnMotor.ConfigAllSettings(constants.m_turnMotorConfig);
    m_turnEncoder.ConfigAllSettings(constants.m_encoderConfig);

    m_driveMotor.EnableVoltageCompensation(true);

    m_turnMotor.SetSelectedSensorPosition(0);
    m_driveMotor.SetSelectedSensorPosition(0);

    //m_driveFeedForward.kS = constants::swerveConstants::DriveKS;
    //m_driveFeedForward.kV = constants::swerveConstants::DriveKV;
    //m_driveFeedForward.kA = constants::swerveConstants::DriveKA;
 
}

void SwerveModule::Set(frc::SwerveModuleState goal, bool optimize){
    //frc::SwerveModuleState sp ;
    frc::Rotation2d currentHeading = GetAngle();//get angle from 1:1 wheel angle encoder

    if(optimize){//why would you not optimize? honestly...
        goal = goal.Optimize(goal, currentHeading);  //just do it already
    }

    //calculate new wheel speed velocity target in falcon units
    double wheelSpeed = ctreHelpers::MPS_2_TalonFX(goal.speed, constants::swerveConstants::DriveGearRatio,
                                                    constants::swerveConstants::WheelDiameter);
    
    //calculate how much the wheel needs to turn
    double deltaDeg = frc::Rotation2d(goal.angle-currentHeading).Degrees().value();//check for sign error Here
    //use the onboard reletive encoder at 1khz PID control and leverage the gain of gear ratio by calculating 
    //the turn based on the current position and the delta ticks.  delta ticks = delta_degress*ticks_per_rev*gear_ratio/360deg
    double turnTarget = m_turnMotor.GetSelectedSensorPosition()+deltaDeg*(2048.0*constants::swerveConstants::TurnGearRatio/360.0);//check for sign error here
    
    //feed forward
    //double ff = m_driveFeedForward.Calculate(goal.speed).value()/constants::NominalVoltage;

    //update setpoints
    m_turnMotor.Set(motorcontrol::ControlMode::MotionMagic, turnTarget); 
    m_driveMotor.Set(ControlMode::Velocity, wheelSpeed);
    //m_driveMotor.Set(ControlMode::Velocity, wheelSpeed, DemandType_ArbitraryFeedForward, ff);
    
}


frc::SwerveModuleState SwerveModule::GetState(){

    units::meters_per_second_t driveMPS(ctreHelpers::TalonFX_2_MPS(m_driveMotor.GetSelectedSensorVelocity(), 
            constants::swerveConstants::DriveGearRatio,
            constants::swerveConstants::WheelDiameter));
    
    
    return {driveMPS, GetAngle()};
}

units::meters_per_second_t SwerveModule::GetSpeedMPS(){
    return GetState().speed;
}

units::degree_t SwerveModule::GetOffsetAngle(){
    
    double encoderRaw = m_turnEncoder.ConfigGetParameter(ParamEnum::eMagnetOffset, 0); //Get Offset of encoder 
    units::degree_t angle(encoderRaw); //convert to degrees
    return angle;//is this function done?????
}

frc::Rotation2d SwerveModule::GetAngle(){

    double encoderRaw = m_turnEncoder.GetAbsolutePosition(); //Get encoder value
    units::degree_t angle(encoderRaw); //convert to degrees
    return frc::Rotation2d(angle); //return rotation object
}



void SwerveModule::InitSendable(wpi::SendableBuilder& builder){

    builder.SetSmartDashboardType("Swerve Module");
    
    //drive motor
    builder.AddDoubleProperty("drive mps", [this] { return GetState().speed.value(); }, nullptr);
    builder.AddDoubleProperty("raw drive speed", [this] { return m_driveMotor.GetSelectedSensorVelocity(); }, nullptr);
    //builder.AddDoubleProperty("raw drive position", [this] { return m_driveMotor.GetSelectedSensorPosition(); }, nullptr);
    builder.AddStringProperty("Drive Control Mode", [this] { return ctreHelpers::CTREControlMode2Str(m_driveMotor.GetControlMode()); }, nullptr);
    //builder.AddDoubleProperty("raw drive pid target", [this] { return ctreHelpers::CTRE_Get_PID_Target(m_driveMotor); }, nullptr);
    //builder.AddDoubleProperty("raw drive pid error", [this] { return ctreHelpers::CTRE_Get_PID_Error(m_driveMotor); }, nullptr);
    
    //turn motor
    //builder.AddDoubleProperty("raw turn speed", [this] { return m_turnMotor.GetSelectedSensorVelocity(); }, nullptr);
    builder.AddDoubleProperty("raw turn position", [this] { return m_turnMotor.GetSelectedSensorPosition(); }, nullptr);
    builder.AddStringProperty("Turn Control Mode", [this] { return ctreHelpers::CTREControlMode2Str(m_turnMotor.GetControlMode()); }, nullptr);
    //builder.AddDoubleProperty("raw turn pid target", [this] { return ctreHelpers::CTRE_Get_PID_Target(m_turnMotor); }, nullptr);
    //builder.AddDoubleProperty("raw turn pid error", [this] { return ctreHelpers::CTRE_Get_PID_Error(m_turnMotor); }, nullptr);

    //turn encoder
    builder.AddDoubleProperty("heading", [this] { return GetState().angle.Degrees().value(); }, nullptr);
    //builder.AddDoubleProperty("raw turn encoder position", [this] { return m_turnEncoder.GetAbsolutePosition(); }, nullptr);
    
}
void SwerveModule::SendData(){
    //drive motor
    frc::SmartDashboard::PutNumber(m_id + " drive mps", GetState().speed.value());
    //frc::SmartDashboard::PutNumber(m_id + " raw drive speed", m_driveMotor.GetSelectedSensorVelocity());
    //frc::SmartDashboard::PutNumber(m_id + " raw drive position", m_driveMotor.GetSelectedSensorPosition());
    //frc::SmartDashboard::PutNumber(m_id + " raw drive Output %", m_driveMotor.GetMotorOutputPercent());
    //frc::SmartDashboard::PutString(m_id + " Drive Control Mode", ctreHelpers::CTREControlMode2Str(m_driveMotor.GetControlMode()));
    frc::SmartDashboard::PutNumber(m_id + " raw drive pid target", ctreHelpers::CTRE_Get_PID_Target(m_driveMotor));
    frc::SmartDashboard::PutNumber(m_id + " raw drive pid error", ctreHelpers::CTRE_Get_PID_Error(m_driveMotor));
    
    //turn motor
    //frc::SmartDashboard::PutNumber(m_id + " raw turn speed", m_turnMotor.GetSelectedSensorVelocity());
    //frc::SmartDashboard::PutNumber(m_id + " raw turn position", m_turnMotor.GetSelectedSensorPosition());
    //frc::SmartDashboard::PutNumber(m_id + " raw turn Output %", m_turnMotor.GetMotorOutputPercent());
    //frc::SmartDashboard::PutString(m_id + " Turn Control Mode", ctreHelpers::CTREControlMode2Str(m_turnMotor.GetControlMode()));
    frc::SmartDashboard::PutNumber(m_id + " raw turn pid target", ctreHelpers::CTRE_Get_PID_Target(m_turnMotor));
    frc::SmartDashboard::PutNumber(m_id + " raw turn pid error", ctreHelpers::CTRE_Get_PID_Error(m_turnMotor));

    //turn encoder
    frc::SmartDashboard::PutNumber(m_id + " heading", GetState().angle.Degrees().value());
    //frc::SmartDashboard::PutNumber(m_id + " raw turn encoder position", m_turnEncoder.GetAbsolutePosition());
    
}