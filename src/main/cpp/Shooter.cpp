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
    //-2316 rear limit
    //2028 forward limit
    

    fmc.slot0.kP = 0.03;
    fmc.slot0.kF = .05;
    
    bmc.slot0.kP = 0.03;
    bmc.slot0.kF = .05;

    
    m_frontMotor.ConfigAllSettings(fmc);
    m_backMotor.ConfigAllSettings(bmc);
    
    m_frontMotor.SetInverted(true);
    m_backMotor.SetInverted(true);
    
    
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

void Shooter::SendData(){
    frc::SmartDashboard::PutNumber("Shooter Front Speed", m_frontMotor.GetSelectedSensorVelocity());
    if(m_frontMotor.GetControlMode()==ControlMode::Velocity){
        frc::SmartDashboard::PutNumber("Shooter Front Error", m_frontMotor.GetClosedLoopError());
    }

    frc::SmartDashboard::PutNumber("Shooter Back Speed", m_backMotor.GetSelectedSensorVelocity());
    if(m_backMotor.GetControlMode()==ControlMode::Velocity){
        frc::SmartDashboard::PutNumber("Shooter Back Error", m_backMotor.GetClosedLoopError());
    }
    
    frc::SmartDashboard::PutNumber("Shooter Front fps", GetFrontFPS().value());

    frc::SmartDashboard::PutNumber("Shooter Back fps", GetBackFPS().value());
    
}