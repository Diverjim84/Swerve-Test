// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <ctre/Phoenix.h>
#include <units/units.h>

class Shooter {
  private:
  TalonFX m_backMotor{22};
  TalonFX m_frontMotor{21};
  TalonSRX m_turnMotor{23};

 public:
  Shooter();

  void SetShooterSpeed(double frontSpeed, double backSpeed);
  void SetAngleMotorSpeed(double speed);
  void SetAngle(units::degree_t angle);
  units::degree_t GetAngle();

  void SendData();

};
