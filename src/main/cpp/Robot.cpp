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

  enum AutoRoutine {
      k5Ball,
      k7Ball,
      k2Ball,
      k1Ball,
  } m_autoSelected;

  frc::SendableChooser<AutoRoutine> m_autoChooser;
  const std::string kAuto5Ball = "5 Ball";
  const std::string kAuto2Ball = "2 Ball";
  const std::string kAuto1Ball = "1 Ball";
  const std::string kAuto7Ball = "7 Ball";
  //std::string m_autoSelected;
  frc::SendableChooser<bool> m_autoGoalChooser;
  const std::string kAutoHigh = "High Goal";
  const std::string kAutoLow  = "Low Goal";
  
  

  frc::Trajectory traj;
  frc::Trajectory traj2;
  frc::Trajectory traj3;
  frc::Trajectory traj4;
  frc::Trajectory traj5;
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

  void Gen5Ball_Old(){
    //set Starting Pose
    frc::Pose2d x(316.15_in, 112.2_in, frc::Rotation2d(-111_deg));
    m_swerve.SetPose(x);
    
    //init trajector selector
    trajSelect = 0;

    //create trajectory 1 - starting to ball 1
    frc::Pose2d trajStartPoint{x.X(),x.Y(),x.Rotation().Degrees()};
    frc::Pose2d trajEndPoint{301_in,10_in,-100_deg};
    std::vector<frc::Translation2d> interiorWaypoints{
      frc::Translation2d{306_in, 71_in}};

    //set speed scaler
    units::scalar_t scaleSpeed = 1.0;
    //configure traj with speed and acceleration 
    units::meters_per_second_t maxV(constants::swerveConstants::MaxSpeed/scaleSpeed);
    units::meters_per_second_squared_t maxA(constants::swerveConstants::MaxAcceleration);
    frc::TrajectoryConfig config{ maxV, maxA};
    
    //don't care for swerve
    config.SetReversed(false);
    
    //gen traj
    traj = frc::TrajectoryGenerator::GenerateTrajectory(trajStartPoint, interiorWaypoints, trajEndPoint, config);

    //traj 2 ball 1 to ball 2&3
    frc::Pose2d trajEndPoint2{0_in,20_in,-100_deg};
    std::vector<frc::Translation2d> interiorWaypoints2{
      frc::Translation2d{270_in, 90_in},
      frc::Translation2d{198_in, 75_in}/*,
      frc::Translation2d{50_in, 50_in}*/
      };
    //create a slow down 
    frc::RectangularRegionConstraint slowRegion1(frc::Translation2d{180_in, 60_in},
                                                 frc::Translation2d{212_in, 90_in},
                                                 frc::MaxVelocityConstraint{1_mps}
                                                );
    config.AddConstraint(slowRegion1);

    traj2 = frc::TrajectoryGenerator::GenerateTrajectory(trajEndPoint, interiorWaypoints2, trajEndPoint2 , config);

  }

  void Gen5Ball(){
    //set Starting Pose
    frc::Pose2d x(316.15_in, 112.2_in, frc::Rotation2d(-111_deg));
    m_swerve.SetPose(x);
    
    //init trajector selector
    trajSelect = 0;

    //create trajectory 1 - starting to ball 1
    frc::Pose2d trajStartPoint{x.X(),x.Y(),x.Rotation().Degrees()};
    frc::Pose2d Ball2{208_in,82_in, 0_deg};//  208,82
    std::vector<frc::Translation2d> waypoints_to_Ball1_and_Ball2{
      frc::Translation2d{301_in, 5_in},
      frc::Translation2d{270_in, 90_in}
      };

    //slowdown region for Ball 1
    frc::RectangularRegionConstraint slowRegionBall1(frc::Translation2d{290_in, 20_in},
                                                 frc::Translation2d{312_in, 0_in},
                                                 frc::MaxVelocityConstraint{.5_mps}
                                                );
    
    //configure traj with speed and acceleration 
    units::meters_per_second_t maxV(constants::swerveConstants::MaxSpeed);
    units::meters_per_second_squared_t maxA(constants::swerveConstants::MaxAcceleration);
    frc::TrajectoryConfig config{ maxV, maxA};
    
    config.AddConstraint(slowRegionBall1);//Add Slow Region

    //gen traj
    traj = frc::TrajectoryGenerator::GenerateTrajectory(trajStartPoint, waypoints_to_Ball1_and_Ball2, Ball2, config);

    //traj 2 ball 1 to ball 2&3
    frc::Pose2d Ball3{0_in,20_in,0_deg};
    std::vector<frc::Translation2d> MidPointBall2ToBall3{
      frc::Translation2d{82_in, 77_in}
      };
    //create a slow down 
    

    traj2 = frc::TrajectoryGenerator::GenerateTrajectory(Ball2, MidPointBall2ToBall3, Ball3 , config);

  }

  void Gen7Ball(){
    //set Starting Pose
    frc::Pose2d x(316.15_in, 112.2_in, frc::Rotation2d(-111_deg));
    m_swerve.SetPose(x);
    
    //init trajector selector
    trajSelect = 0;

    //create trajectory 1 - starting to ball 1
    frc::Pose2d trajStartPoint{x.X(),x.Y(),x.Rotation().Degrees()};
    frc::Pose2d Ball2{208_in,82_in, 0_deg};//  208,82
    std::vector<frc::Translation2d> waypoints_to_Ball1_and_Ball2{
      frc::Translation2d{301_in, 5_in},
      frc::Translation2d{270_in, 90_in}
      };

    //slowdown region for Ball 1
    frc::RectangularRegionConstraint slowRegionBall1(frc::Translation2d{290_in, 20_in},
                                                 frc::Translation2d{312_in, 0_in},
                                                 frc::MaxVelocityConstraint{.5_mps}
                                                );
    
    //configure traj with speed and acceleration 
    units::meters_per_second_t maxV(constants::swerveConstants::MaxSpeed);
    units::meters_per_second_squared_t maxA(constants::swerveConstants::MaxAcceleration);
    frc::TrajectoryConfig config{ maxV, maxA};
    
    config.AddConstraint(slowRegionBall1);//Add Slow Region

    //gen traj
    traj = frc::TrajectoryGenerator::GenerateTrajectory(trajStartPoint, waypoints_to_Ball1_and_Ball2, Ball2, config);


    frc::Pose2d Ball5{0_in,80_in,0_deg};
    std::vector<frc::Translation2d> MidPointBall2ToBall5{
      frc::Translation2d{82_in, 87_in}
      };
    
    traj2 = frc::TrajectoryGenerator::GenerateTrajectory(Ball2, MidPointBall2ToBall5, Ball5 , config);
    traj3 = frc::TrajectoryGenerator::GenerateTrajectory(Ball5, MidPointBall2ToBall5, Ball2 , config);

    //traj 2 ball 1 to ball 2&3
    frc::Pose2d Ball3{0_in,20_in,0_deg};
    std::vector<frc::Translation2d> MidPointBall2ToBall3{
      frc::Translation2d{82_in, 77_in}
      };
  
    traj4 = frc::TrajectoryGenerator::GenerateTrajectory(Ball2, MidPointBall2ToBall3, Ball3 , config);
    traj5 = frc::TrajectoryGenerator::GenerateTrajectory(Ball3, MidPointBall2ToBall3, Ball2 , config);

  }

  void Gen2Ball(){
    //set Starting Pose
    frc::Pose2d x(282.4_in, 191.2_in, frc::Rotation2d(159_deg));
    m_swerve.SetPose(x);
    
    //init trajector selector
    trajSelect = 0;

    //create trajectory 1 - starting to ball 1
    frc::Pose2d trajStartPoint{x.X(),x.Y(),x.Rotation().Degrees()};
    frc::Pose2d Ball2{194_in,242_in,159_deg};
    std::vector<frc::Translation2d> interiorWaypoints{
      frc::Translation2d{241_in, 244_in}};

    //set speed scaler
    units::scalar_t scaleSpeed = 1.0;
    //configure traj with speed and acceleration 
    units::meters_per_second_t maxV(constants::swerveConstants::MaxSpeed/scaleSpeed);
    units::meters_per_second_squared_t maxA(constants::swerveConstants::MaxAcceleration);
    frc::TrajectoryConfig config{ maxV, maxA};
    
    //don't care for swerve
    config.SetReversed(false);
    
    //gen traj
    traj = frc::TrajectoryGenerator::GenerateTrajectory(trajStartPoint, interiorWaypoints, Ball2, config);

    
  }

  void Gen1Ball(){
    //set Starting Pose
    frc::Pose2d x(282.4_in, 191.2_in, frc::Rotation2d(159_deg));
    m_swerve.SetPose(x);
    
    //init trajector selector
    trajSelect = 0;

    //create trajectory 1 - starting to ball 1
    frc::Pose2d trajStartPoint{x.X(),x.Y(),x.Rotation().Degrees()};
    frc::Pose2d trajEndPoint{230_in,282_in,159_deg};
    std::vector<frc::Translation2d> interiorWaypoints{
      frc::Translation2d{241_in, 244_in}};

    //set speed scaler
    units::scalar_t scaleSpeed = 1.0;
    //configure traj with speed and acceleration 
    units::meters_per_second_t maxV(constants::swerveConstants::MaxSpeed/scaleSpeed);
    units::meters_per_second_squared_t maxA(constants::swerveConstants::MaxAcceleration);
    frc::TrajectoryConfig config{ maxV, maxA};
    
    //don't care for swerve
    config.SetReversed(false);
    
    //gen traj
    traj = frc::TrajectoryGenerator::GenerateTrajectory(trajStartPoint, interiorWaypoints, trajEndPoint, config);

    
  }

  void GenTraj(){
    AutoRoutine t = m_autoChooser.GetSelected();
    if(t==m_autoSelected){
      return;
    }

    m_autoSelected = t;
    switch(m_autoSelected){
      case AutoRoutine::k1Ball : Gen1Ball();
        break;
      case AutoRoutine::k2Ball : Gen2Ball();
        break;
      case AutoRoutine::k5Ball : Gen5Ball();
        break;
      case AutoRoutine::k7Ball : Gen7Ball();
        break;

    }
  }

  void Run5Ball(){
    frc::Pose2d p;
    switch(trajSelect){
      case 0: if(m_autoGoalChooser.GetSelected()){
                shooter.SetShooter(constants::ShooterFront_NearHigh , constants::ShooterBack_NearHigh);
              }else{
                shooter.SetShooter(constants::ShooterFront_NearLow, constants::ShooterBack_NearLow);
              }
              indexer.Fire();
              indexer.Update();
              if(autoTimer.Get()>.5_s){
                trajSelect++;
                autoTimer.Reset();
              }
              break;
      case 1: p = traj.Sample(autoTimer.Get()).pose; 
              m_swerve.DrivePos(p.X(), p.Y(), m_swerve.GetHeading().Degrees()-GetTargetAngleDelta());
              shooter.SetShooter(40_fps, 45_fps);//Get shooter Up to Speed
              indexer.Update();
              if(autoTimer.Get()>(traj.TotalTime())){
                trajSelect++;
                autoTimer.Reset();
              }
              break;
      case 2: shooter.SetShooter(constants::ShooterFront_Auto, constants::ShooterBack_Auto);
              indexer.SetUpper(1.0);
              indexer.SetLower(.6);
              if((!indexer.GetLowerBallSensor()&&!indexer.GetLowerBallSensor())||autoTimer.Get()>1_s){
                trajSelect++;
                autoTimer.Reset();
              }
              break;
      case 3: p = traj2.Sample(autoTimer.Get()).pose;
              m_swerve.DrivePos(p.X(), p.Y(), m_swerve.GetHeading().Degrees()-GetTargetAngleDelta());
              indexer.Update();
              if(autoTimer.Get()>(traj2.TotalTime()+1_s)){
                trajSelect++;
                autoTimer.Reset();
              }
              break;
      case 4: p = traj3.Sample(autoTimer.Get()).pose;
              m_swerve.DrivePos(p.X(), p.Y(), m_swerve.GetHeading().Degrees()-GetTargetAngleDelta());
              indexer.Update();
              if(autoTimer.Get()>(traj3.TotalTime()+1_s)){
                trajSelect++;
                autoTimer.Reset();
              }
              break;
      case 5: shooter.SetShooter(constants::ShooterFront_Auto, constants::ShooterBack_Auto);
              indexer.SetUpper(1.0);
              indexer.SetLower(.6);
              if((!indexer.GetLowerBallSensor()&&!indexer.GetLowerBallSensor())||autoTimer.Get()>1_s){
                trajSelect++;
                autoTimer.Reset();
              }
              break;
      default: m_swerve.DriveXY(0_mps,0_mps);
    }
    
    
  }

  void Run5Ball_Old(){
    frc::Pose2d p;
    switch(trajSelect){
      case 0: if(m_autoGoalChooser.GetSelected()){
                shooter.SetShooter(constants::ShooterFront_NearHigh , constants::ShooterBack_NearHigh);
              }else{
                shooter.SetShooter(constants::ShooterFront_NearLow, constants::ShooterBack_NearLow);
              }
              indexer.Fire();
              if(autoTimer.Get()>.5_s){
                trajSelect++;
                autoTimer.Reset();
              }
              break;
      case 1: p = traj.Sample(autoTimer.Get()).pose; 
              m_swerve.DrivePos(p.X(), p.Y(), m_swerve.GetHeading().Degrees()-GetTargetAngleDelta());
              if(autoTimer.Get()>(traj.TotalTime())){
                trajSelect++;
                autoTimer.Reset();
              }
              break;
      case 2: shooter.SetShooter(constants::ShooterFront_Auto, constants::ShooterBack_Auto);
              //indexer.Fire();
              indexer.SetUpper(1.0);
              indexer.SetLower(.6);
              if(autoTimer.Get()>1_s){
                trajSelect++;
                autoTimer.Reset();
              }
              break;
      case 3: p = traj2.Sample(autoTimer.Get()).pose;
              m_swerve.DrivePos(p.X(), p.Y(), m_swerve.GetHeading().Degrees()-GetTargetAngleDelta());
              if(autoTimer.Get()>(traj2.TotalTime()+1_s)){
                trajSelect++;
                autoTimer.Reset();
              }
              break;
      default: m_swerve.DriveXY(0_mps,0_mps);
    }
    if(IsTargetVisable())
    {
      m_swerve.SetTargetHeading(m_swerve.GetHeading().Degrees()-GetTargetAngleDelta()/1.0);
    }
    
  }

  void Run7Ball(){
    frc::Pose2d p;
    switch(trajSelect){
      case 0: if(m_autoGoalChooser.GetSelected()){
                shooter.SetShooter(constants::ShooterFront_NearHigh , constants::ShooterBack_NearHigh);
              }else{
                shooter.SetShooter(constants::ShooterFront_NearLow, constants::ShooterBack_NearLow);
              }
              indexer.Fire();
              indexer.Update();
              if(autoTimer.Get()>.5_s){
                trajSelect++;
                autoTimer.Reset();
              }
              break;
      case 1: p = traj.Sample(autoTimer.Get()).pose; 
              m_swerve.DrivePos(p.X(), p.Y(), m_swerve.GetHeading().Degrees()-GetTargetAngleDelta());
              if(autoTimer.Get()>(traj.TotalTime()+2_s)){
                trajSelect++;
                autoTimer.Reset();
              }
              break;
      case 2: shooter.SetShooter(constants::ShooterFront_Auto, constants::ShooterBack_Auto);
              indexer.SetUpper(1.0);
              indexer.SetLower(.6);
              if((!indexer.GetLowerBallSensor()&&!indexer.GetLowerBallSensor())||autoTimer.Get()>1_s){
                trajSelect++;
                autoTimer.Reset();
              }
              break;
      case 3: p = traj2.Sample(autoTimer.Get()).pose;
              m_swerve.DrivePos(p.X(), p.Y(), 0_deg);//Need to approach Straight because of wall
              if(autoTimer.Get()>(traj2.TotalTime()+2_s)){
                trajSelect++;
                autoTimer.Reset();
              }
              break;
      case 4: p = traj3.Sample(autoTimer.Get()).pose;
              m_swerve.DrivePos(p.X(), p.Y(), m_swerve.GetHeading().Degrees()-GetTargetAngleDelta());
              if(autoTimer.Get()>(traj3.TotalTime()+0_s)){
                trajSelect++;
                autoTimer.Reset();
              }
              break;
      case 5: shooter.SetShooter(constants::ShooterFront_Auto, constants::ShooterBack_Auto);
              indexer.Fire();
              if((!indexer.GetLowerBallSensor()&&!indexer.GetLowerBallSensor())||autoTimer.Get()>1_s){
                trajSelect++;
                autoTimer.Reset();
              }
              break;
      case 6: p = traj4.Sample(autoTimer.Get()).pose;
              m_swerve.DrivePos(p.X(), p.Y(), 45_deg);//approach corner wall
              if(autoTimer.Get()>(traj4.TotalTime()+3_s)){
                trajSelect++;
                autoTimer.Reset();
              }
              break;
      case 7: p = traj5.Sample(autoTimer.Get()).pose;
              m_swerve.DrivePos(p.X(), p.Y(), m_swerve.GetHeading().Degrees()-GetTargetAngleDelta());
              if(autoTimer.Get()>(traj5.TotalTime()+0_s)){
                trajSelect++;
                autoTimer.Reset();
              }
              break;
      case 8: shooter.SetShooter(constants::ShooterFront_Auto, constants::ShooterBack_Auto);
              indexer.Fire();
              if((!indexer.GetLowerBallSensor()&&!indexer.GetLowerBallSensor())||autoTimer.Get()>1_s){
                trajSelect++;
                autoTimer.Reset();
              }
              break;
      default: m_swerve.DriveXY(0_mps,0_mps);
    }
  }

  void Run2Ball(){
    frc::Pose2d p;
    switch(trajSelect){
      case 0: if(m_autoGoalChooser.GetSelected()){
                shooter.SetShooter(constants::ShooterFront_NearHigh , constants::ShooterBack_NearHigh);
              }else{
                shooter.SetShooter(constants::ShooterFront_NearLow, constants::ShooterBack_NearLow);
              }
              indexer.Fire();
              if(autoTimer.Get()>.5_s){
                trajSelect++;
                autoTimer.Reset();
              }
              break;
      case 1: p = traj.Sample(autoTimer.Get()).pose; 
              m_swerve.DrivePos(p.X(), p.Y(), m_swerve.GetHeading().Degrees()-GetTargetAngleDelta());
              if(autoTimer.Get()>(traj.TotalTime())){
                trajSelect++;
                autoTimer.Reset();
              }
              break;
      case 2: shooter.SetShooter(constants::ShooterFront_Auto, constants::ShooterBack_Auto);
              indexer.Fire();
              if(autoTimer.Get()>2_s){
                trajSelect++;
                autoTimer.Reset();
              }
              break;
      default: m_swerve.DriveXY(0_mps,0_mps);
               m_swerve.SetTargetHeading(m_swerve.GetHeading().Degrees()-GetTargetAngleDelta()/1.0);
    }
  }

  void Run1Ball(){
    frc::Pose2d p;
    switch(trajSelect){
      case 0: if(m_autoGoalChooser.GetSelected()){
                shooter.SetShooter(constants::ShooterFront_NearHigh , constants::ShooterBack_NearHigh);
              }else{
                shooter.SetShooter(constants::ShooterFront_NearLow, constants::ShooterBack_NearLow);
              }
              indexer.Fire();
              if(autoTimer.Get()>.5_s){
                trajSelect++;
                autoTimer.Reset();
              }
              break;
      case 1: p = traj.Sample(autoTimer.Get()).pose; 
              m_swerve.DrivePos(p.X(), p.Y(), m_swerve.GetHeading().Degrees()-GetTargetAngleDelta());
              if(autoTimer.Get()>(traj.TotalTime())){
                trajSelect++;
                autoTimer.Reset();
              }
              break;
      default: m_swerve.DriveXY(0_mps,0_mps);
               m_swerve.SetTargetHeading(m_swerve.GetHeading().Degrees()-GetTargetAngleDelta()/1.0);
    }
  }

  


 public:
  void RobotInit() override {
    //frc::SmartDashboard::PutData("Drivetrain", &m_swerve);
    //m_swerve.PutChildSendables();
    
    m_autoChooser.SetDefaultOption(kAuto5Ball, AutoRoutine::k5Ball);
    m_autoChooser.AddOption(kAuto7Ball, AutoRoutine::k7Ball);
    m_autoChooser.AddOption(kAuto2Ball, AutoRoutine::k2Ball);
    m_autoChooser.AddOption(kAuto1Ball, AutoRoutine::k1Ball);
    frc::SmartDashboard::PutData("Auto Modes", &m_autoChooser);

    m_autoGoalChooser.SetDefaultOption(kAutoLow, false);
    m_autoGoalChooser.SetDefaultOption(kAutoHigh, true);

    m_swerve.SetHeading(0_deg);
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

  void DisabledPeriodic() override {
    GenTraj();
  }

  void AutonomousInit() override {
    //GenTraj();// make sure no last minute routine change
/*
    switch(frc::DriverStation::GetAlliance()){
      case frc::DriverStation::kBlue: indexer.Init(Indexer::BallColor::kBlue);
                                      break;
      case frc::DriverStation::kRed: indexer.Init(Indexer::BallColor::kRed);
                                      break;
      default: indexer.Init(Indexer::BallColor::kBlue);
    }
*/
    frc::Pose2d x(316.15_in, 112.2_in, frc::Rotation2d(-111_deg));
     
    m_swerve.SetPose(x);
    autoTimer.Reset();
    autoTimer.Start();
    trajSelect = 0;
    
    Gen5Ball();
    m_autoSelected = AutoRoutine::k5Ball;
    indexer.SetFeederSpeed(.7);
    indexer.SetFeederPos(true);
  }

  void AutonomousPeriodic() override {
    //DriveWithJoystick(false);
    m_swerve.UpdateOdometry();
    indexer.Update();
    switch(m_autoSelected){
      case AutoRoutine::k1Ball : Run1Ball();
        break;
      case AutoRoutine::k2Ball : Run2Ball();
        break;
      case AutoRoutine::k5Ball : Run5Ball();
        break;
      case AutoRoutine::k7Ball : Run7Ball();
        break;
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
    m_swerve.SetTargetHeading(m_swerve.GetHeading().Degrees());
    indexer.SetFeederPos(true);

    
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
    indexer.Update(codriver.GetStartButton());//Codriver Eject all balls

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
    if((fabs(drivex)+fabs(drivey))/2.0<.05){
      drivex = 0.0;
      drivey = 0.0;
    }

    //Get Right joystick for heading and velocity control
    double hy = -driver.GetRightX(); //hy is w for vel control
    double hx = -driver.GetRightY();

    units::scalar_t scale = 1.0-driver.GetRightTriggerAxis()*.9;
    scale = scale*.8;
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
        if(sqrt(hx*hx+hy*hy) > .95)//make sure the joystick is begin used by calculating magnitude
        {
            m_swerve.SetTargetHeading(frc::Rotation2d(hx, hy).Degrees());
        }
        m_swerve.DriveXY(
            m_xspeedLimiter.Calculate(pow(drivex,1)) * constants::swerveConstants::MaxSpeed * scale,
            m_yspeedLimiter.Calculate(pow(drivey,1)) * constants::swerveConstants::MaxSpeed * scale);
        break;
      case DriveMode::TargetTracking :
        //add code to set heading angle
        if(IsTargetVisable())
        {
          auto v = m_swerve.GetChassisSpeeds();
          auto d = GetTargetRange();
          
          m_swerve.SetTargetHeading(m_swerve.GetHeading().Degrees()-GetTargetAngleDelta());
        }
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
      shooter.SetShooter(25_fps, 37_fps);//@20deg 27, 37 is solid
      manualShooterMode = false;
    }
    //Near Low Goal Shot
    if(codriver.GetXButtonPressed()){
      shooter.SetShooter(20_fps, 15_fps);//@20deg 20, 15 is solid
      manualShooterMode = false;
    }
    //Mid High Goal Shot
    if(codriver.GetBButtonPressed()){
      shooter.SetShooter(40_fps, 45_fps);
      manualShooterMode = false;
    }
    //Auto High Goal Shot
    if(codriver.GetYButtonPressed()){
      shooter.SetShooter(100_fps, 0_fps);
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

      

    if(driver.GetBackButtonPressed()){
      m_swerve.SetPose(frc::Pose2d(0_m, 0_m, frc::Rotation2d(0_deg)));
    }

    
    Balls();
    //indexer.SetFeederSpeed(.8);
    //indexer.SetUpper(1.0);
    //indexer.SetLower(.6);
    Shoot();
    Drive();
    
  }

 
};

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif
