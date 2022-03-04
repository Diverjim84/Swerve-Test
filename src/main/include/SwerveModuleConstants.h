// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once


#include <ctre/Phoenix.h>



class SwerveModuleConstants {
 public:
  int m_driveMotorID;
  int m_turnMotorID;
  int m_encoderID;
  
  ctre::phoenix::motorcontrol::can::TalonFXConfiguration m_driveMotorConfig;
  ctre::phoenix::motorcontrol::can::TalonFXConfiguration m_turnMotorConfig;
  ctre::phoenix::sensors::CANCoderConfiguration m_encoderConfig;

  SwerveModuleConstants(int driveID, int turnID, int encoderID):
    m_driveMotorConfig(),
    m_turnMotorConfig(),
    m_encoderConfig()
  {
    m_driveMotorID = driveID;
    m_turnMotorID = turnID;
    m_encoderID = encoderID;
    //m_angleOffset = angleOffset;

    

  };
};
