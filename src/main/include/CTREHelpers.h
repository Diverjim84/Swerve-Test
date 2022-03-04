// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once
#include <units/units.h>
#include <units/math.h>
#include <wpi/numbers>
#include <ctre/phoenix/motorcontrol/ControlMode.h>

namespace ctreHelpers{
  constexpr double TalonFX_2_MPS(double falconVel, double gearRatio, units::meter_t wheelDiameter){
    //Falcon Vel is in ticks per 100ms, so vel*10 for ticks per sec
    //(ticks per sec)/ticks per rev(2048)= motor rev per sec
    //motor rev per sec/gear ratio = wheel rev per sec
    //wheel rev per sec * wheel circumference (diameter*pi) = meters per sec
    return wpi::numbers::pi*wheelDiameter.value()*((falconVel*10.0/2048.0)/gearRatio);
  }

  constexpr double MPS_2_TalonFX(units::meters_per_second_t vel, double gearRatio, units::meter_t wheelDiameter_meters){
    //auto wps = vel/(wpi::numbers::pi*wheelDiameter_meters); //find wheel rev per sec
    //auto rps = wps*gearRatio; //find motor rev per sec
    //auto tp100ms = 2048.0*rps/10.0; //find ticks per 100ms
    return 2048.0*(gearRatio*(vel.value()/(wpi::numbers::pi*wheelDiameter_meters.value())))/10.0;
  }

  inline double CTRE_Get_PID_Error(ctre::phoenix::motorcontrol::can::BaseMotorController &motor)
  {
    ctre::phoenix::motorcontrol::ControlMode mode = motor.GetControlMode();
    if( mode == ControlMode::Velocity || 
        mode == ControlMode::Position || 
        mode == ControlMode::MotionMagic ||
        mode == ControlMode::MotionProfile ||
        mode == ControlMode::MotionProfileArc )
    {
      return motor.GetClosedLoopError();
    }else
    { 
      return 0.0;
    }
    return 0.0;
  }

  inline double CTRE_Get_PID_Target(ctre::phoenix::motorcontrol::can::BaseMotorController &motor)
  {
    ctre::phoenix::motorcontrol::ControlMode mode = motor.GetControlMode();
    if( mode == ControlMode::Velocity || 
        mode == ControlMode::Position || 
        mode == ControlMode::MotionMagic ||
        mode == ControlMode::MotionProfile ||
        mode == ControlMode::MotionProfileArc )
    {
      return motor.GetClosedLoopTarget();
    }else
    { 
      return 0.0;
    }
    return 0.0;
  }

  inline std::string CTREControlMode2Str(ctre::phoenix::motorcontrol::ControlMode mode){
    switch(mode){
      case ctre::phoenix::motorcontrol::ControlMode::PercentOutput: return "Percent Output";
      case ctre::phoenix::motorcontrol::ControlMode::Position : return "Position PID";
      case ctre::phoenix::motorcontrol::ControlMode::Velocity: return "Velocity PID";
      case ctre::phoenix::motorcontrol::ControlMode::MotionMagic: return "Motion Magic";
      case ctre::phoenix::motorcontrol::ControlMode::MotionProfile: return "Motion Profile";
      case ctre::phoenix::motorcontrol::ControlMode::MotionProfileArc: return "Motion Profile Arc";
      case ctre::phoenix::motorcontrol::ControlMode::Follower : return "Follower";
      case ctre::phoenix::motorcontrol::ControlMode::Disabled : return "Disabled";
      case ctre::phoenix::motorcontrol::ControlMode::Current : return "Current";
      case ctre::phoenix::motorcontrol::ControlMode::MusicTone : return "MusicTone";
      default: return "";
    }
    return "";
  }

}
