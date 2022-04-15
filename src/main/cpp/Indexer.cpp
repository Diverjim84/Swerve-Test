// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Indexer.h"
#include <units/time.h>
#include <frc/smartdashboard/SmartDashboard.h>

Indexer::Indexer(){
    m_feeder.SetInverted(true);
    m_lowerStageMotor.SetInverted(true);
    m_upperStageMotor.SetInverted(false);

    m_matcher.AddColorMatch(kRedBall);
    m_matcher.AddColorMatch(kBlueBall);
    
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
    
    if(GetUpperBallSensor()){
        m_upperState.color = m_myColor;
        m_upperState.state = BallState::kReady;
    }else{
        m_upperState.color = BallColor::kNone;
        m_upperState.state = BallState::kEmpty;
    }

    if(GetLowerBallSensor()){
        m_lowerState.color = m_myColor;
        m_lowerState.state = BallState::kReady;
    }else{
        m_lowerState.color = BallColor::kNone;
        m_lowerState.state = BallState::kEmpty;
    }
}

void Indexer::Update(bool eject){
    if(eject)
    {
        SetUpper(-1.0);
        SetLower(-1.0);
        SetFeederSpeed(-1.0);
        if(GetUpperBallSensor()){
            m_upperState.state = kReady;
        }else{
            m_upperState.state = kEmpty;
        }
        if(GetLowerBallSensor()){
            m_lowerState.state = kReady;
        }else{
            m_lowerState.state = kEmpty;
        }
        return;
    }
    /*
    if(m_firingTimer.Get()>.2_s){
            m_upperState.state = BallState::kEmpty;
            m_upperState.color = BallColor::kNone;
            m_firing = false;
            //SetUpper(0.5);//MOVE!!!
        }else{
            m_upperState.state = BallState::kEmpty;
        }
    */
    if(GetUpperBallSensor()){
        if(m_upperState.state != kFiring){
            m_upperState.state = kReady;
        }
    }else{
        if(m_upperState.state == kFiring){
            //if(m_firingTimer.Get()>.25_s){
                m_upperState.state = BallState::kEmpty;
                m_upperState.color = BallColor::kNone;
                m_firing = false;
            //}else{
            //    m_upperState.state = kFiring;
            //}
        }else{
            m_upperState.state = BallState::kEmpty;
        }
    }

    if(GetLowerBallSensor()){
        if(m_upperState.state == kEmpty){
             m_lowerState.state = kEmpty;
        }else{
            m_lowerState.state = kReady;
        }
    }else{
        m_lowerState.state = kEmpty;
    }

    if(m_feederPiston.Get() == frc::DoubleSolenoid::Value::kReverse && m_firingTimer.Get()>2_s){
        if(m_upperState.state == BallState::kEmpty && m_lowerState.state == BallState::kEmpty){
            Stop();
            return;
        }
        
    }else{
        switch (m_upperState.state)
        {
        case BallState::kEmpty : 
            SetUpper(.2);//was .3
            break;
        case BallState::kFiring : 
            SetUpper(1.0);//Increase after Mechanical Fixes their crap!!!
            break;
        case BallState::kReady : 
            SetUpper(.0);
            break;
        default:
            break;
        }
        switch (m_lowerState.state)
        {
        case BallState::kEmpty : 
            SetLower(.5);
            //SetFeederSpeed(.5);
            break;
        case BallState::kReady : 
            SetLower(.0);
            break;
        default:
            break;
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
    m_firing = true;
    m_firingTimer.Reset();
    m_firingTimer.Start();
    m_upperState.state = BallState::kFiring;
    SetUpper(1.0);
    if(m_lowerState.state == kReady){
        SetLower(.5);
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

void Indexer::SendData(){
    
    frc::SmartDashboard::PutBoolean("Upper Sensor", GetUpperBallSensor());
    frc::SmartDashboard::PutBoolean("Lower Sensor", GetLowerBallSensor());

    frc::Color c = m_colorSensor.GetColor();
    frc::SmartDashboard::PutNumber("Color Sensor Red", c.red);
    frc::SmartDashboard::PutNumber("Color Sensor Blue", c.blue);
    frc::SmartDashboard::PutNumber("Color Sensor Green", c.green);
    
    switch (m_upperState.state)
    {
    case BallState::kEmpty : frc::SmartDashboard::PutString("Upper State", "Empty");
        break;
    case BallState::kFiring : frc::SmartDashboard::PutString("Upper State", "Firing");
        break;
    case BallState::kReady : frc::SmartDashboard::PutString("Upper State", "Ready");
        break;
    default:
        frc::SmartDashboard::PutString("Upper State", "Unknown");
        break;
    }

    switch (m_lowerState.state)
    {
    case BallState::kEmpty : frc::SmartDashboard::PutString("Lower State", "Empty");
        break;
    case BallState::kFiring : frc::SmartDashboard::PutString("Lower State", "Firing");
        break;
    case BallState::kReady : frc::SmartDashboard::PutString("Lower State", "Ready");
        break;
    default:
        frc::SmartDashboard::PutString("Lower State", "Unknown");
        break;
    }

}