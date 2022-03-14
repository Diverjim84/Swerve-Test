// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Indexer.h"
#include <units/time.h>

Indexer::Indexer(){
    m_feeder.SetInverted(false);
    m_lowerStageMotor.SetInverted(false);
    m_upperStageMotor.SetInverted(false);
    //Init(BallColor::kNone);
    m_feederPiston.Set(frc::DoubleSolenoid::Value::kReverse);
}

void Indexer::Init(BallColor myColor){
    m_myColor = myColor;

    m_firing = false;
    m_firingTimer.Stop();
    m_firingTimer.Reset();

    m_feederUpTimer.Stop();
    m_feederUpTimer.Reset();
    
    if(m_upperBallSensor.Get()){
        m_upperState.color = m_myColor;
        m_upperState.state = BallState::kReady;
    }else{
        m_upperState.color = BallColor::kNone;
        m_upperState.state = BallState::kEmpty;
    }

    if(m_lowerBallSensor.Get()){
        m_lowerState.color = m_myColor;
        m_lowerState.state = BallState::kReady;
    }else{
        m_lowerState.color = BallColor::kNone;
        m_lowerState.state = BallState::kEmpty;
    }
}

void Indexer::Update(){
    if(m_upperBallSensor.Get() && m_upperState.state == BallState::kEmpty){
        m_upperState.state = BallState::kReady;
        SetUpper(0.0);
    }
    if(!m_upperBallSensor.Get() && m_upperState.state == BallState::kFiring){
        if(m_firingTimer.Get()>.2_s){
            m_upperState.state = BallState::kEmpty;
            m_upperState.color = BallColor::kNone;
            SetUpper(0.5);
        }
    }
    if(m_lowerBallSensor.Get() && 
        m_lowerState.state == BallState::kEmpty &&
        m_upperState.state == BallState::kReady)
    {
        m_lowerState.state = BallState::kReady;
        SetLower(0.0);
    }
    if(m_upperState.state == BallState::kEmpty && m_lowerState.state == BallState::kReady){
        m_lowerState.state = BallState::kEmpty;
        SetLower(.6);
    }

    if(m_feederPiston.Get() == frc::DoubleSolenoid::Value::kReverse && m_firingTimer.Get()>1.5_s){
        if(m_upperState.state == BallState::kEmpty && m_lowerState.state == BallState::kEmpty){
            Stop();
        }
        
    }
}

void Indexer::Stop(){
    m_feeder.Set(0.0);
    m_lowerStageMotor.Set(0.0);
    m_upperStageMotor.Set(0.0);
}

int  Indexer::Ready2Fire(){
    int balls = 0;
    if(m_upperState.state == BallState::kReady && m_upperState.color == m_myColor){
        balls++;
        if(m_lowerState.state == BallState::kReady && m_lowerState.color == m_myColor){
            balls++;
        }
        return balls;
    }else{
        if(m_upperState.state == BallState::kReady && m_upperState.color != m_myColor){
            return -1;
        }
    }
}

void Indexer::Fire(){
    if(m_firing){
        return;
    }
    if(m_upperState.state == BallState::kReady){
        m_firing = true;
        m_firingTimer.Start();
        m_upperState.state = BallState::kFiring;
        SetUpper(1.0);
    }

}

void Indexer::SetFeeder(bool down, double speed){
    SetFeederPos(down);
    SetFeederSpeed(speed);
}

void Indexer::SetFeederPos(bool down){
    if(down){
        m_feederUpTimer.Stop();
        m_feederUpTimer.Reset();
        m_feederPiston.Set(frc::DoubleSolenoid::Value::kForward);
    }else{
        m_feederUpTimer.Start();
        m_feederPiston.Set(frc::DoubleSolenoid::Value::kReverse);
    }
}
void Indexer::ToggleFeederPos(){
    if(m_feederPiston.Get() == frc::DoubleSolenoid::Value::kForward){
        SetFeederPos(false);
    }else{
        SetFeederPos(true);
    }
}

void Indexer::SetFeederSpeed(double speed){
    m_feeder.Set(speed);
}

void Indexer::SetLower(double speed){
    m_lowerStageMotor.Set(speed);
}

void Indexer::SetUpper(double speed){
    m_upperStageMotor.Set(speed);
}
