// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/GenericHID.h>

/*
  A = 3
  B = 2
  X = 4
  Y = 1
  LBumper = 5
  RBumper = 6
  LTrigger = 7
  RTrigger = 8
  - = 9
  + = 10
  LJoystickButton = 11
  RJoystickButton = 12
  home = 13
  circle = 14

  LX = Axis 0
  LY = Axis 1
  RX = Axis 2
  RY = Axis 3

 * /

class GameCubeController : public frc::GenericHID {
 public:
  GameCubeController(int port);
};

//*/