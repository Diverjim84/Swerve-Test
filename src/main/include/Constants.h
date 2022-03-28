// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <units/math.h>
#include <units/units.h>

#include <frc/kinematics/SwerveDriveKinematics.h>

#include <ctre/Phoenix.h>

#include "SwerveModuleConstants.h"

namespace constants{

  constexpr double NominalVoltage = 12.0;
  constexpr double ShooterTurnTicksPerDegree = 4096.0/(360.0/6.0);

  constexpr units::scalar_t ShooterGearRatio = 1.0;
  constexpr units::inch_t ShooterWheelRadius = 4_in;

  namespace swerveConstants{

    constexpr int PigeonID = 32;
    constexpr bool PigeonInverted = false;

    constexpr double RotGain = 1.5;//for controlling the heading closed loop
    constexpr double PosGain = 5.0;//for controlling the robot auto position

    /* Drivetrain Constants */
    constexpr units::meter_t WheelBaseWidth = 19.35_in;
    constexpr units::meter_t WheelBaseLength = 24.35_in;

    constexpr units::meter_t WheelDiameter = 3.94_in;
      
    constexpr double DriveGearRatio = 6.75;
    constexpr double TurnGearRatio = 12.8;

    constexpr bool DriveRightInvert = true;
    constexpr bool DriveLeftInvert = false;
    
    /* Swerve Profiling Values */
    constexpr units::meters_per_second_t MaxSpeed{4.5}; //meters per second
    constexpr units::meters_per_second_squared_t MaxAcceleration{6.0}; //meters per second
    constexpr units::radians_per_second_t MaxAngularVelocity{11.5};

    /* Drive Motor Characterization Values */
    constexpr auto DriveKS = 0.66707_V; 
    constexpr auto DriveKV = (2.7887_V / 1_mps);
    constexpr auto DriveKA = (0.29537_V / 1_mps_sq); 

    struct SwerveConfig{

      frc::Translation2d m_frontLeftLocation{ WheelBaseLength/2.0, WheelBaseWidth/2.0 };
      frc::Translation2d m_frontRightLocation{ WheelBaseLength/2.0, -WheelBaseWidth/2.0 };
      frc::Translation2d m_rearLeftLocation{ -WheelBaseLength/2.0, WheelBaseWidth/2.0 };
      frc::Translation2d m_rearRightLocation{ -WheelBaseLength/2.0, -WheelBaseWidth/2.0 };

      

      /* Neutral Modes */
      NeutralMode angleNeutralMode = NeutralMode::Coast;
      NeutralMode driveNeutralMode = NeutralMode::Brake;

      
      /* Angle Encoder Invert */
      bool canCoderInvert = false;

      TalonFXConfiguration driveMotorConfig;
      TalonFXConfiguration turnMotorConfig;
      
      phoenix::motorcontrol::SupplyCurrentLimitConfiguration driveMotorCurrentLimits;
      phoenix::motorcontrol::SupplyCurrentLimitConfiguration turnMotorCurrentLimits;

      CANCoderConfiguration encoderConfig;

                              //driveID, turnID, encoderID, angleOffset
      SwerveModuleConstants frontLeftConstants{1, 3, 13};
      SwerveModuleConstants frontRightConstants{7, 8, 12};
      SwerveModuleConstants rearLeftConstants{5, 6, 14};
      SwerveModuleConstants rearRightConstants{2, 4, 15};

      SwerveConfig()
      {

        driveMotorConfig.slot0.kP = .05;
        driveMotorConfig.slot0.kF = .046;

        driveMotorConfig.slot0.integralZone = 100.0;
        driveMotorConfig.slot0.maxIntegralAccumulator = 100.0;

        driveMotorConfig.initializationStrategy = phoenix::sensors::SensorInitializationStrategy::BootToZero;

        turnMotorConfig.slot0.kP = .1;
        //turnMotorConfig.slot0.kD = 12.0;
        turnMotorConfig.slot0.kF = .05;
        turnMotorConfig.motionCruiseVelocity = 19000.0;
        turnMotorConfig.motionAcceleration = 30000.0;

        turnMotorConfig.initializationStrategy = phoenix::sensors::SensorInitializationStrategy::BootToZero;

        encoderConfig.absoluteSensorRange = phoenix::sensors::AbsoluteSensorRange::Signed_PlusMinus180;
        encoderConfig.sensorDirection = canCoderInvert;
        encoderConfig.initializationStrategy = phoenix::sensors::SensorInitializationStrategy::BootToAbsolutePosition;
        encoderConfig.sensorTimeBase = phoenix::sensors::SensorTimeBase::PerSecond;

        /* Swerve Current Limiting */
        turnMotorCurrentLimits.currentLimit = 25;
        turnMotorCurrentLimits.triggerThresholdCurrent = 40;
        turnMotorCurrentLimits.triggerThresholdTime = 0.1;
        turnMotorCurrentLimits.enable = true;

        driveMotorCurrentLimits.currentLimit = 35;
        driveMotorCurrentLimits.triggerThresholdCurrent = 60;
        driveMotorCurrentLimits.triggerThresholdTime = 0.1;
        driveMotorCurrentLimits.enable = true;

        driveMotorConfig.voltageCompSaturation = constants::NominalVoltage;

        driveMotorConfig.supplyCurrLimit = driveMotorCurrentLimits;
        turnMotorConfig.supplyCurrLimit = turnMotorCurrentLimits;

        frontLeftConstants.m_driveMotorConfig = driveMotorConfig;
        frontRightConstants.m_driveMotorConfig = driveMotorConfig;
        rearLeftConstants.m_driveMotorConfig = driveMotorConfig;
        rearRightConstants.m_driveMotorConfig = driveMotorConfig;

        frontLeftConstants.m_driveMotorConfig.slot0.kP = .15;
        frontRightConstants.m_driveMotorConfig.slot0.kP = .15;
        rearLeftConstants.m_driveMotorConfig.slot0.kP = .15;
        rearRightConstants.m_driveMotorConfig.slot0.kP = .15;

        frontLeftConstants.m_driveMotorConfig.slot0.kP = .15;
        frontRightConstants.m_driveMotorConfig.slot0.kP = .15;
        rearLeftConstants.m_driveMotorConfig.slot0.kP = .15;
        rearRightConstants.m_driveMotorConfig.slot0.kP = .15;

        frontLeftConstants.m_turnMotorConfig = turnMotorConfig;
        frontRightConstants.m_turnMotorConfig = turnMotorConfig;
        rearLeftConstants.m_turnMotorConfig = turnMotorConfig;
        rearRightConstants.m_turnMotorConfig = turnMotorConfig;

        frontLeftConstants.m_encoderConfig = encoderConfig;
        frontRightConstants.m_encoderConfig = encoderConfig;
        rearLeftConstants.m_encoderConfig = encoderConfig;
        rearRightConstants.m_encoderConfig = encoderConfig;

        //update with offset angle from calibration [-180,180)
        //NOTE:  multiply angle at 0 heading times -1 to get offset
        frontLeftConstants.m_encoderConfig.magnetOffsetDegrees = -137.9;
        frontRightConstants.m_encoderConfig.magnetOffsetDegrees = 167.8;
        rearLeftConstants.m_encoderConfig.magnetOffsetDegrees = -17.45;
        rearRightConstants.m_encoderConfig.magnetOffsetDegrees = 72.06;

      };
    };
  }

}