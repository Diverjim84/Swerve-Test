// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <rev/CANSparkMax.h>
#include <frc/DigitalInput.h>
#include <frc/DoubleSolenoid.h>
#include <frc/Timer.h>

class Indexer {
public:
  enum BallState{
    kEmpty,
    kReady,
    kFiring
  };

  enum BallColor{
    kNone,
    kBlue,
    kRed
  };
  struct StageState{
    BallState state;
    BallColor color;
  };

private:
  rev::CANSparkMax m_upperStageMotor{2, rev::CANSparkMaxLowLevel::MotorType::kBrushed};
  rev::CANSparkMax m_lowerStageMotor{3, rev::CANSparkMaxLowLevel::MotorType::kBrushless};
  rev::CANSparkMax m_feeder{4, rev::CANSparkMaxLowLevel::MotorType::kBrushless};

  frc::DigitalInput m_upperBallSensor{1};
  frc::DigitalInput m_lowerBallSensor{0};

  BallColor m_myColor;
  StageState m_lowerState;
  StageState m_upperState;

  frc::DoubleSolenoid m_feederPiston{frc::PneumaticsModuleType::CTREPCM,0,1};
  frc::Timer m_feederUpTimer;

  frc::Timer m_firingTimer;
  bool m_firing;
  

 public:
  
  
  Indexer();
  void Init(BallColor myColor);
  void Update();
  void Stop();
  int  Ready2Fire();
  void Fire();
  
  void SetFeeder(bool down, double speed);
  void SetFeederPos(bool down);
  void SetFeederSpeed(double speed);
  void ToggleFeederPos();

  void SetLower(double speed);
  void SetUpper(double speed);
  
  BallColor GetBalLColor(){return m_myColor;};

};
