// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Shooter.h"
#include <frc/smartdashboard/SmartDashboard.h>
#include "Constants.h"
#include "CTREHelpers.h"
#include <units/units.h>

Shooter::Shooter(){
    


    TalonFXConfiguration fmc;
    TalonFXConfiguration bmc;
    TalonSRXConfiguration tmc;
    //-2316 rear limit
    //2028 forward limit
    

    fmc.slot0.kP = 0.0;
    fmc.slot0.kF = .055;
    
    bmc.slot0.kP = 0.0;
    bmc.slot0.kF = .055;

    //turn gear ratio 72:12

    tmc.slot0.kP = 0.60;
    tmc.slot0.kI = 0.0;
    tmc.slot0.kD = 0.20;

    tmc.slot0.integralZone = 200;
    tmc.slot0.maxIntegralAccumulator = 100;
    tmc.slot0.allowableClosedloopError = 50;
    

    tmc.forwardLimitSwitchNormal = motorcontrol::LimitSwitchNormal_NormallyClosed;
    tmc.reverseLimitSwitchNormal = motorcontrol::LimitSwitchNormal_NormallyClosed;
    
    tmc.forwardSoftLimitThreshold = 1900;
    tmc.reverseSoftLimitThreshold = -1900;
    tmc.reverseSoftLimitEnable = true;
    tmc.forwardSoftLimitEnable = true;

    m_frontMotor.ConfigAllSettings(fmc);
    m_backMotor.ConfigAllSettings(bmc);
    m_turnMotor.ConfigAllSettings(tmc);

    m_frontMotor.SetInverted(true);
    m_backMotor.SetInverted(true);
    m_turnMotor.SetInverted(false);

    m_turnMotor.ConfigSelectedFeedbackSensor(FeedbackDevice::CTRE_MagEncoder_Absolute,0,30);
    double offset = m_turnMotor.GetSensorCollection().GetPulseWidthPosition();
    offset = 2532.0 - offset ;
    m_turnMotor.SetSelectedSensorPosition(offset, 0);
    //0 = 2532

}

void Shooter::SetShooterSpeed(double frontSpeed, double backSpeed){
    //m_frontMotor.Set(motorcontrol::ControlMode::Velocity, frontSpeed);
    //m_backMotor.Set(motorcontrol::ControlMode::Velocity, backSpeed);
    m_frontMotor.Set(motorcontrol::ControlMode::PercentOutput, frontSpeed);
    m_backMotor.Set(motorcontrol::ControlMode::PercentOutput, backSpeed);
}

void Shooter::SetShooter(units::feet_per_second_t frontSpeed, units::feet_per_second_t backSpeed){
    m_frontMotor.Set(motorcontrol::ControlMode::Velocity, 
        ctreHelpers::MPS_2_TalonFX(frontSpeed, constants::ShooterGearRatio, constants::ShooterWheelRadius));
    m_backMotor.Set(motorcontrol::ControlMode::Velocity, 
        ctreHelpers::MPS_2_TalonFX(backSpeed, constants::ShooterGearRatio, constants::ShooterWheelRadius));
}

void Shooter::SetAngleMotorSpeed(double speed){
    m_turnMotor.Set(motorcontrol::ControlMode::PercentOutput, speed);
}

void Shooter::SetAngle(units::degree_t angle){
    double ticks = angle.value()*constants::ShooterTurnTicksPerDegree;
    m_turnMotor.Set(ControlMode::Position, ticks);
}

units::degree_t Shooter::GetAngle(){
    double ticks = m_turnMotor.GetSensorCollection().GetQuadraturePosition();
    return units::degree_t(ticks/constants::ShooterTurnTicksPerDegree);
}

void Shooter::SendData(){
    frc::SmartDashboard::PutNumber("Shooter Front Speed", m_frontMotor.GetSelectedSensorVelocity());
    if(m_frontMotor.GetControlMode()==ControlMode::Velocity){
        frc::SmartDashboard::PutNumber("Shooter Front Error", m_frontMotor.GetClosedLoopError());
    }

    frc::SmartDashboard::PutNumber("Shooter Back Speed", m_backMotor.GetSelectedSensorVelocity());
    if(m_backMotor.GetControlMode()==ControlMode::Velocity){
        frc::SmartDashboard::PutNumber("Shooter Back Error", m_backMotor.GetClosedLoopError());
    }
    frc::SmartDashboard::PutNumber("Shooter Turn Position", m_turnMotor.GetSelectedSensorPosition());
    frc::SmartDashboard::PutBoolean("Shooter Turn Front LM", m_turnMotor.IsFwdLimitSwitchClosed());
    frc::SmartDashboard::PutBoolean("Shooter Turn Rear LM", m_turnMotor.IsRevLimitSwitchClosed());
    frc::SmartDashboard::PutNumber("Shooter Angle Error", m_turnMotor.GetClosedLoopError());
    frc::SmartDashboard::PutNumber("Shooter Angle SP", m_turnMotor.GetClosedLoopTarget());
    frc::SmartDashboard::PutNumber("Shooter Angle", GetAngle().value());
    frc::SmartDashboard::PutNumber("Shooter Angle Output", m_turnMotor.GetMotorOutputPercent());

}