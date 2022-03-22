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
#include <frc/trajectory/constraint/RectangularRegionConstraint.h>
#include <frc/trajectory/constraint/MaxVelocityConstraint.h>
#include <frc/smartdashboard/SendableChooser.h>

#include <networktables/NetworkTable.h>
#include "networktables/NetworkTableEntry.h"
#include "networktables/NetworkTableInstance.h"
#include "networktables/NetworkTableValue.h"

#include <frc/DriverStation.h>

#include "Drivetrain.h"
#include "SwerveModule.h"
#include "Constants.h"
#include <frc/SerialPort.h>
#include <networktables/NetworkTable.h>

#include "Indexer.h"
#include "Shooter.h"

class Robot : public frc::TimedRobot {
private:
  frc::XboxController driver{0};
  frc::XboxController codriver{1};

  constants::swerveConstants::SwerveConfig config;
  Drivetrain m_swerve{config};
  bool headingControl;

  Indexer indexer;
  Shooter shooter;

  double m_speedScale;

  frc::SendableChooser<std::string> m_autoChooser;
  const std::string kAuto5Ball = "5 Ball";
  const std::string kAuto2Ball = "2 Ball";
  const std::string kAuto1Ball = "1 Ball";
  const std::string kAuto7Ball = "7 Ball";
  std::string m_autoSelected;

  frc::Trajectory traj;
  frc::Trajectory traj2;
  frc::Timer autoTimer;
  int trajSelect;

  std::shared_ptr<nt::NetworkTable> limelight = nt::NetworkTableInstance::GetDefault().GetTable("limelight");
  
  enum DriveMode{
    VelocityMode,
    HeadingControl,
    TargetTracking
  } driveMode;

  bool manualShooterMode;

  // Slew rate limiters to make joystick inputs more gentle; 1/3 sec from 0
  // to 1.
  frc::SlewRateLimiter<units::scalar> m_xspeedLimiter{2 / 1_s};
  frc::SlewRateLimiter<units::scalar> m_yspeedLimiter{2 / 1_s};
  frc::SlewRateLimiter<units::scalar> m_rotLimiter{2 / 1_s};

  void DriveWithJoystick(bool fieldRelative) {

    m_speedScale = 1.0-driver.GetRightTriggerAxis();

    // Get the x speed. We are inverting this because Xbox controllers return
    // negative values when we push forward.
    const auto xSpeed = -m_xspeedLimiter.Calculate(
                            frc::ApplyDeadband(driver.GetLeftY(), 0.15)) *
                        constants::swerveConstants::MaxSpeed*m_speedScale;

    // Get the y speed or sideways/strafe speed. We are inverting this because
    // we want a positive value when we pull to the left. Xbox controllers
    // return positive values when you pull to the right by default.
    const auto ySpeed = -m_yspeedLimiter.Calculate(
                            frc::ApplyDeadband(driver.GetLeftX(), 0.15)) *
                        constants::swerveConstants::MaxSpeed*m_speedScale;

    

    frc::SmartDashboard::PutNumber("Drive Command X Speed", xSpeed.value());
    frc::SmartDashboard::PutNumber("Drive Command Y Speed", ySpeed.value());
    
    m_swerve.DriveXY(xSpeed, ySpeed);
    
  }

  void Gen5Ball(){
    frc::Pose2d x(316.15_in, 112.2_in, frc::Rotation2d(-111_deg));
     
    m_speedScale = 1.0;

    m_swerve.SetPose(x);
    trajSelect = 0;

    frc::Pose2d trajStartPoint{x.X(),x.Y(),x.Rotation().Degrees()};
    frc::Pose2d trajEndPoint{301_in,20_in,-100_deg};
    std::vector<frc::Translation2d> interiorWaypoints{
      frc::Translation2d{306_in, 71_in}};

    units::scalar_t scaleSpeed = 1.0;
    units::meters_per_second_t maxV(constants::swerveConstants::MaxSpeed/scaleSpeed);
    units::meters_per_second_squared_t maxA(constants::swerveConstants::MaxAcceleration);
    frc::TrajectoryConfig config{ maxV, maxA};

    config.SetReversed(false);
    
    traj = frc::TrajectoryGenerator::GenerateTrajectory(trajStartPoint, interiorWaypoints, trajEndPoint, config);

    frc::Pose2d trajEndPoint2{12_in,33_in,-100_deg};
    std::vector<frc::Translation2d> interiorWaypoints2{
      frc::Translation2d{270_in, 90_in},
      frc::Translation2d{198_in, 75_in}/*,
      frc::Translation2d{50_in, 50_in}*/
      };
    
    frc::RectangularRegionConstraint slowRegion1(frc::Translation2d{180_in, 60_in},
                                                 frc::Translation2d{212_in, 90_in},
                                                 frc::MaxVelocityConstraint{1_mps}
                                                );
    config.AddConstraint(slowRegion1);

    traj2 = frc::TrajectoryGenerator::GenerateTrajectory(trajEndPoint, interiorWaypoints2, trajEndPoint2 , config);

  }

  void Gen7Ball(){

  }

  void Gen2Ball(){
    
  }

  void Gen1Ball(){
    
  }

  void Run5Ball(){
    
  }

  void Run7Ball(){
    
  }

  void Run2Ball(){
    
  }

  void Run1Ball(){
    
  }

  


 public:
  void RobotInit() override {
    //frc::SmartDashboard::PutData("Drivetrain", &m_swerve);
    //m_swerve.PutChildSendables();
    
    m_autoChooser.SetDefaultOption(kAuto5Ball, kAuto5Ball);
    m_autoChooser.AddOption(kAuto7Ball, kAuto7Ball);
    m_autoChooser.AddOption(kAuto2Ball, kAuto2Ball);
    m_autoChooser.AddOption(kAuto1Ball, kAuto1Ball);
    frc::SmartDashboard::PutData("Auto Modes", &m_autoChooser);

    m_swerve.SetHeading(-160_deg);
    headingControl =  true;
    manualShooterMode = false;
    driveMode = DriveMode::HeadingControl;
    

  }

  bool IsTargetVisable(){
    
    double tv = limelight->GetNumber("tv", 0.0);
    if(tv== 0.0){
      return false;
    }else{
      return true;
    }
    
   return false;
  }

  units::degree_t GetTargetAngleDelta(){
    
    double tx = limelight->GetNumber("tx",0.0);
    if(IsTargetVisable()){
      return units::degree_t(tx);
    }else{
      return 0_deg;
    }
    
   return 0_deg;
  }

  units::inch_t GetTargetRange(){
    
    double targetOffsetAngle_Vertical = limelight->GetNumber("ty", 0.0);
    
    // how many degrees back is your limelight rotated from perfectly vertical?
    double limelightMountAngleDegrees = 75.0;

    // distance from the center of the Limelight lens to the floor
    double limelightHeightInches = 20.0;

    // distance from the target to the floor
    double goalHeightInches = 104.0;

    double angleToGoalDegrees = limelightMountAngleDegrees + targetOffsetAngle_Vertical;
    double angleToGoalRadians = angleToGoalDegrees * (3.14159 / 180.0);

    //calculate distance
    double distanceFromLimelightToGoalInches = (goalHeightInches - limelightHeightInches)/tan(angleToGoalRadians);
    return units::inch_t(distanceFromLimelightToGoalInches);
    //*/return 0_in;
  }


  void RobotPeriodic() override {
    m_swerve.UpdateOdometry();

    m_swerve.SendData();
    indexer.SendData();
    shooter.SendData();

    frc::SmartDashboard::PutNumber("Stick Left Y", driver.GetLeftY());
    frc::SmartDashboard::PutNumber("Stick Left X", driver.GetLeftX());

    frc::SmartDashboard::PutNumber("Stick Right Y", driver.GetRightY());
    frc::SmartDashboard::PutNumber("Stick Right X", driver.GetRightX());

    frc::SmartDashboard::PutNumber("Vision Target Distance (inch)", GetTargetRange().value());
    frc::SmartDashboard::PutNumber("Vision Target Angle (deg)", GetTargetAngleDelta().value());
    frc::SmartDashboard::PutBoolean("Vision Target Visable", IsTargetVisable());

    //devModule.SendData();

  }

  void AutonomousInit() override {
    switch(frc::DriverStation::GetAlliance()){
      case frc::DriverStation::kBlue: indexer.Init(Indexer::BallColor::kBlue);
                                      break;
      case frc::DriverStation::kRed: indexer.Init(Indexer::BallColor::kRed);
                                      break;
      default: indexer.Init(Indexer::BallColor::kBlue);
    }

    frc::Pose2d x(316.15_in, 112.2_in, frc::Rotation2d(-111_deg));
     
    m_swerve.SetPose(x);
    autoTimer.Reset();
    autoTimer.Start();
    trajSelect = 0;

  }

  void AutonomousPeriodic() override {
    //DriveWithJoystick(false);
    m_swerve.UpdateOdometry();
    
    
    frc::Pose2d p;
    switch(trajSelect){
      case 0: p = traj.Sample(autoTimer.Get()).pose; 
              m_swerve.DrivePos(p.X(), p.Y(), -100_deg);
              if(autoTimer.Get()>(traj.TotalTime()+1_s)){
                trajSelect++;
                autoTimer.Reset();
              }
              break;
      case 1: p = traj2.Sample(autoTimer.Get()).pose;
              m_swerve.DrivePos(p.X(), p.Y(), -140_deg);
              if(autoTimer.Get()>(traj2.TotalTime()+1_s)){
                trajSelect++;
                autoTimer.Reset();
              }
              break;
      default: m_swerve.DriveXY(0_mps,0_mps);
    }
    
    
  }

  void TeleopInit(){
    if(indexer.GetBalLColor()== Indexer::BallColor::kNone){
      switch(frc::DriverStation::GetAlliance()){
      case frc::DriverStation::kBlue: indexer.Init(Indexer::BallColor::kBlue);
                                      break;
      case frc::DriverStation::kRed: indexer.Init(Indexer::BallColor::kRed);
                                      break;
      default: indexer.Init(Indexer::BallColor::kBlue);
    }
    }
  }

  void Balls(){
    //Driver Left Bumper Toggles Feeder up/down
    if(codriver.GetLeftBumperPressed()){
      indexer.ToggleFeederPos();
    }
    //Driver Left Trigger Drives Intake speed
    double feedSpeed = codriver.GetLeftTriggerAxis();
    if(feedSpeed <.1){
      indexer.SetFeederSpeed(0.0);
    }else{
      //set a min feed speed and then scale the input speed based on the remaining range
      double minFeed = .25;
      feedSpeed = minFeed+feedSpeed*(1.0-minFeed);
      indexer.SetFeederSpeed(feedSpeed);
    }
    //update Indexer state maching and control the lower and upper stage motors
    indexer.Update();

    //codriver Start button ejects balls
    if(codriver.GetStartButton()){
      indexer.SetLower(-.3);
      indexer.SetUpper(-.3);
      indexer.SetFeederSpeed(-.3);
    }

    if(codriver.GetBackButtonPressed())
    {
      indexer.Stop();
      shooter.SetShooterSpeed(0.0, 0.0);
    }


  }

  void Drive()
  {
    double drivex = -driver.GetLeftY();
    double drivey = -driver.GetLeftX();
    if((fabs(drivex)+fabs(drivey))/2.0<.1){
      drivex = 0.0;
      drivey = 0.0;
    }

    //Get Right joystick for heading and velocity control
    double hy = -driver.GetRightX(); //hy is w for vel control
    double hx = -driver.GetRightY();

    units::scalar_t scale = 1.0-driver.GetRightTriggerAxis()*.5;

    if(driver.GetXButtonPressed()){
      driveMode = DriveMode::TargetTracking;
    }
    
    if(driver.GetAButtonPressed()){
      driveMode = DriveMode::HeadingControl;
    }
    
    if(driver.GetBButtonPressed()){
      driveMode = DriveMode::VelocityMode;
    }

    switch(driveMode){
      case DriveMode::VelocityMode : 
        if(fabs(hy)<.1){//deadband rot vel, translations were done
          hy = 0.0;
        }
        m_swerve.Drive(
            m_xspeedLimiter.Calculate(pow(drivex,1)) * constants::swerveConstants::MaxSpeed * scale,
            m_yspeedLimiter.Calculate(pow(drivey,1)) * constants::swerveConstants::MaxSpeed * scale, 
            m_rotLimiter.Calculate(pow(hy,3)) * constants::swerveConstants::MaxAngularVelocity * scale, 
            true);
        break;
      case DriveMode::HeadingControl :
        if(sqrt(hx*hx+hy*hy) > .75)//make sure the joystick is begin used by calculating magnitude
        {
            m_swerve.SetTargetHeading(frc::Rotation2d(hx, hy).Degrees());
        }
        m_swerve.DriveXY(
            m_xspeedLimiter.Calculate(pow(drivex,1)) * constants::swerveConstants::MaxSpeed * scale,
            m_yspeedLimiter.Calculate(pow(drivey,1)) * constants::swerveConstants::MaxSpeed * scale);
        break;
      case DriveMode::TargetTracking :
        //add code to set heading angle
        m_swerve.DriveXY(
            m_xspeedLimiter.Calculate(pow(drivex,1)) * constants::swerveConstants::MaxSpeed * scale,
            m_yspeedLimiter.Calculate(pow(drivey,1)) * constants::swerveConstants::MaxSpeed * scale);
        break;
       
    }
    /*
    if(!headingControl){
      if(fabs(hy)<.1){
        hy = 0.0;
      }
      m_swerve.Drive( constants::swerveConstants::MaxSpeed*scale*drivex, 
                    constants::swerveConstants::MaxSpeed*scale*drivey, 
                    constants::swerveConstants::MaxAngularVelocity*scale*hy, true);
    }else{
      if(sqrt(hx*hx+hy*hy) > .75)//make sure the joystick is begin used by calculating magnitude
      {
          m_swerve.SetTargetHeading(frc::Rotation2d(hx, hy).Degrees());
      }
      DriveWithJoystick(true);
      
    }
    */

  }

  void Shoot(){
    //Codriver right & left Bumper send fire command
    if(codriver.GetRightBumperPressed()){
      indexer.Fire();
    }

    double backRoller = -frc::ApplyDeadband(codriver.GetRightY(), .15);
    double frontRoller = -frc::ApplyDeadband(codriver.GetLeftY(), .15);
    //see if codriver is trying to manually control shooter
    if(fabs(backRoller) >.1 || fabs(frontRoller)> .1){
      manualShooterMode = true;
    }

    //If in manual mode, Drive shooter manually
    if(manualShooterMode){
      shooter.SetShooterSpeed(frontRoller, backRoller);
    }

    //Near High Goal Shot
    if(codriver.GetAButtonPressed()){
      //exit velocity of 21fps, 75deg, 1.25ft from target edge
      shooter.SetShooter(60_fps, 80_fps);
      manualShooterMode = false;
    }
    //Near Low Goal Shot
    if(codriver.GetXButtonPressed()){
      shooter.SetShooter(20_fps, 10_fps);
      manualShooterMode = false;
    }
    //Mid High Goal Shot
    if(codriver.GetBButtonPressed()){
      shooter.SetShooter(50_fps, 40_fps);
      manualShooterMode = false;
    }
    //Auto High Goal Shot
    if(codriver.GetYButtonPressed()){
      shooter.SetShooter(100_fps, 85_fps);
      manualShooterMode = false;
    }


    /*
    if(driver.GetYButtonPressed()){
      shooter.SetAngle(20_deg);
    }
    if(driver.GetAButtonPressed()){
      shooter.SetAngle(-20_deg);
    }
    if(driver.GetBButtonPressed()){
      shooter.SetAngle(0_deg);
    }
    */
  }

  void TeleopPeriodic() override { 
    
    //DriveWithJoystick(true); 

    if(driver.GetStartButtonPressed()){
      headingControl = !headingControl;
    }    

    if(driver.GetBackButtonPressed()){
      m_swerve.SetPose(frc::Pose2d(0_m, 0_m, frc::Rotation2d(0_deg)));
    }

    
    Balls();
    Shoot();
    Drive();
    
  }

 
};

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif
