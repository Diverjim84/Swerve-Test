// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <frc/MathUtil.h>
#include <frc/TimedRobot.h>
#include <frc/XboxController.h>
#include <frc/filter/SlewRateLimiter.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/trajectory/Trajectory.h>
#include <frc/trajectory/TrajectoryGenerator.h>


#include "Drivetrain.h"
#include "SwerveModule.h"
#include "Constants.h"
#include <frc/SerialPort.h>
#include <networktables/NetworkTable.h>

class Robot : public frc::TimedRobot {
private:
  frc::XboxController stick{0};
  constants::swerveConstants::SwerveConfig config;
  Drivetrain m_swerve{config};
  bool headingControl;

  double m_speedScale;

  frc::Trajectory traj;
  frc::Trajectory traj2;
  frc::Timer autoTimer;
  int trajSelect;


  

  // Slew rate limiters to make joystick inputs more gentle; 1/3 sec from 0
  // to 1.
  frc::SlewRateLimiter<units::scalar> m_xspeedLimiter{3 / 1_s};
  frc::SlewRateLimiter<units::scalar> m_yspeedLimiter{3 / 1_s};
  frc::SlewRateLimiter<units::scalar> m_rotLimiter{3 / 1_s};

  void DriveWithJoystick(bool fieldRelative) {

    m_speedScale = 1.0-stick.GetRightTriggerAxis();

    // Get the x speed. We are inverting this because Xbox controllers return
    // negative values when we push forward.
    const auto xSpeed = -m_xspeedLimiter.Calculate(
                            frc::ApplyDeadband(stick.GetLeftY(), 0.15)) *
                        constants::swerveConstants::MaxSpeed*m_speedScale;

    // Get the y speed or sideways/strafe speed. We are inverting this because
    // we want a positive value when we pull to the left. Xbox controllers
    // return positive values when you pull to the right by default.
    const auto ySpeed = -m_yspeedLimiter.Calculate(
                            frc::ApplyDeadband(stick.GetLeftX(), 0.15)) *
                        constants::swerveConstants::MaxSpeed*m_speedScale;

    

    frc::SmartDashboard::PutNumber("Drive Command X Speed", xSpeed.value());
    frc::SmartDashboard::PutNumber("Drive Command Y Speed", ySpeed.value());
    
    m_swerve.DriveXY(xSpeed, ySpeed);
    
  }


 public:
  void RobotInit() override {
    //frc::SmartDashboard::PutData("Drivetrain", &m_swerve);
    //m_swerve.PutChildSendables();
    
    m_swerve.SetHeading(0_deg);
    headingControl =  true;
    frc::Pose2d x(0_m, 0_m, frc::Rotation2d(0_deg));
     
    m_speedScale = 1.0;

    m_swerve.SetPose(x);
    trajSelect = 0;

    frc::Pose2d trajStartPoint{0_ft,0_ft,0_deg};
    frc::Pose2d trajEndPoint{12_ft,-6_ft,0_deg};
    std::vector<frc::Translation2d> interiorWaypoints{
      frc::Translation2d{4_ft, 0_ft}};

    units::scalar_t scaleSpeed = 1.0;
    units::meters_per_second_t maxV(constants::swerveConstants::MaxSpeed/scaleSpeed);
    units::meters_per_second_squared_t maxA(constants::swerveConstants::MaxAcceleration);
    frc::TrajectoryConfig config{ maxV, maxA};

    config.SetReversed(false);
    
    traj = frc::TrajectoryGenerator::GenerateTrajectory(trajStartPoint, interiorWaypoints, trajEndPoint, config);
    config.SetReversed(true);
    traj2 = frc::TrajectoryGenerator::GenerateTrajectory(trajEndPoint, interiorWaypoints, trajStartPoint , config);

  }

  void RobotPeriodic() override {
    m_swerve.UpdateOdometry();

    m_swerve.SendData();

    frc::SmartDashboard::PutNumber("Stick Left Y", stick.GetLeftY());
    frc::SmartDashboard::PutNumber("Stick Left X", stick.GetLeftX());

    frc::SmartDashboard::PutNumber("Stick Right Y", stick.GetRightY());
    frc::SmartDashboard::PutNumber("Stick Right X", stick.GetRightX());

    //devModule.SendData();

  }

  void AutonomousInit() override {
    m_swerve.SetPose(frc::Pose2d(0_m,0_m,m_swerve.GetPose().Rotation()));
    autoTimer.Reset();
    autoTimer.Start();
    trajSelect = 0;

  }

  void AutonomousPeriodic() override {
    //DriveWithJoystick(false);
    m_swerve.UpdateOdometry();
    if(autoTimer.Get()>(traj.TotalTime()+1_s)){
      trajSelect++;
      autoTimer.Reset();
    }
    
    frc::Pose2d p;
    switch(trajSelect){
      case 0: p = traj.Sample(autoTimer.Get()).pose; 
              m_swerve.DrivePos(p.X(), p.Y(), -90_deg);
              break;
      case 1: p = traj2.Sample(autoTimer.Get()).pose;
              m_swerve.DrivePos(p.X(), p.Y(), 90_deg);
              break;
      default: m_swerve.DriveXY(0_mps,0_mps);
    }
    
    
  }

  void TeleopPeriodic() override { 
    
    //DriveWithJoystick(true); 

    if(stick.GetStartButtonPressed()){
      headingControl = !headingControl;
    }    

    if(stick.GetBackButtonPressed()){
      m_swerve.SetPose(frc::Pose2d(0_m, 0_m, frc::Rotation2d(0_deg)));
    }

    double drivex = -stick.GetLeftY();
    double drivey = -stick.GetLeftX();
    if((fabs(drivex)+fabs(drivey))/2.0<.2){
      drivex = 0.0;
      drivey = 0.0;
    }
    double hy = -stick.GetRightX();
    double hx = -stick.GetRightY();
    double mag = sqrt(hx*hx+hy*hy);
    

    double w  = -stick.GetRightX();
    


    double scale = 1.0-stick.GetRightTriggerAxis();
    if(!headingControl){
      if(fabs(w)<.2){
        w = 0.0;
      }
      m_swerve.Drive( constants::swerveConstants::MaxSpeed*scale*drivex, 
                    constants::swerveConstants::MaxSpeed*scale*drivey, 
                    constants::swerveConstants::MaxAngularVelocity*scale*w, true);
    }else{
      if(mag > .5)
      {
          m_swerve.SetTargetHeading(frc::Rotation2d(hx, hy).Degrees());
      }
      DriveWithJoystick(true);
      /*
      m_swerve.DriveXY(constants::swerveConstants::MaxSpeed/scale*drivex, 
                      constants::swerveConstants::MaxSpeed/scale*drivey);
      */
    }

    //devModule.Set(g);
    frc::SmartDashboard::PutNumber("input x", drivex);
    frc::SmartDashboard::PutNumber("input y", drivey);
    frc::SmartDashboard::PutNumber("input w", w);
  }

 
};

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif
