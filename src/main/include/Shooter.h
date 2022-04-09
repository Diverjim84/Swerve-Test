// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <ctre/Phoenix.h>
#include <units/units.h>
#include "CTREHelpers.h"
#include "Constants.h"

class Shooter {
  private:
  TalonFX m_backMotor{22};
  TalonFX m_frontMotor{21};
  //TalonSRX m_turnMotor{23};

 public:
  Shooter();

  void SetShooterSpeed(double frontSpeedPercent, double backSpeedPercent);
  void SetShooter(units::feet_per_second_t frontSpeed, units::feet_per_second_t backSpeed);
  
  units::feet_per_second_t GetFrontFPS(){return units::meters_per_second_t( ctreHelpers::TalonFX_2_MPS(m_frontMotor.GetSelectedSensorVelocity(),
                                        constants::ShooterGearRatio, 
                                        constants::ShooterWheelRadius));};
  units::feet_per_second_t GetBackFPS(){return units::meters_per_second_t(ctreHelpers::TalonFX_2_MPS(m_backMotor.GetSelectedSensorVelocity(),
                                        constants::ShooterGearRatio, 
                                        constants::ShooterWheelRadius));};

  void SendData();

};
