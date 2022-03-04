// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "AutoTrajEngine.h"

AutoTrajEngine::AutoTrajEngine() {
    m_running = false;

}

void AutoTrajEngine::Init(int routineSelector){
    //Build traj here
}

void AutoTrajEngine::Start(){
    //Start timer
    m_timer.Start();
    m_running = true;

    //Make sure traj exists 
    
    //Am I missing anything?
}

void AutoTrajEngine::End(){
    //Stop the timer
    m_timer.Stop();
    //set running false
    m_running = false;
    //Anything else?
}
  