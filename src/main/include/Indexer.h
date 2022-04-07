// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <rev/CANSparkMax.h>
#include <frc/DigitalInput.h>
#include <frc/DoubleSolenoid.h>
#include <frc/Timer.h>
#include <rev/ColorSensorV3.h>
#include <rev/ColorMatch.h>
#include <frc/util/color.h>

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
  rev::CANSparkMax m_upperStageMotor{18, rev::CANSparkMaxLowLevel::MotorType::kBrushed};
  rev::CANSparkMax m_lowerStageMotor{19, rev::CANSparkMaxLowLevel::MotorType::kBrushless};
  rev::CANSparkMax m_feeder{20, rev::CANSparkMaxLowLevel::MotorType::kBrushless};

  frc::DigitalInput m_upperBallSensor{2};
  frc::DigitalInput m_lowerBallSensor{0};

  BallColor m_myColor;
  StageState m_lowerState;
  StageState m_upperState;

  frc::DoubleSolenoid m_feederPiston{frc::PneumaticsModuleType::CTREPCM,1,0};
  frc::Timer m_feederUpTimer;

  frc::Timer m_firingTimer;
  bool m_firing;

  rev::ColorSensorV3 m_colorSensor{frc::I2C::Port::kOnboard};
  rev::ColorMatch m_matcher;

  static constexpr frc::Color kBlueBall = frc::Color(0.143, 0.427, 0.429);
  static constexpr frc::Color kRedBall = frc::Color(0.561, 0.232, 0.114);
  

 public:
  
  
  Indexer();
  void Init(BallColor myColor);
  void Update(bool eject = false);
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

  void SendData();

  bool GetUpperBallSensor(){return !m_upperBallSensor.Get();};
  bool GetLowerBallSensor(){return !m_lowerBallSensor.Get();};

  bool IsUpperEmpty(){if(m_upperState.state==kReady){return false;}else{return true;}};
  bool IsLowerEmpty(){if(m_lowerState.state==kReady){return false;}else{return true;}};

};
