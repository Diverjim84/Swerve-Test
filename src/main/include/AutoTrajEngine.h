// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/trajectory/Trajectory.h>
#include <frc/Timer.h>

class AutoTrajEngine {
  private:
  frc::Trajectory m_traj;
  frc::Timer m_timer;

  bool m_running;

 public:
  AutoTrajEngine();

  void Init(int routineSelector);
  void Start();
  bool IsRunning(){return m_running;};
  void End();

};
