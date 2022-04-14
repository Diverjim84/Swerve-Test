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

#include <frc/MathUtil.h>
#include <units/math.h>

#include <networktables/NetworkTable.h>
#include "networktables/NetworkTableEntry.h"
#include "networktables/NetworkTableInstance.h"
#include "networktables/NetworkTableValue.h"

#include <frc/Notifier.h>

#include <frc/DriverStation.h>

#include "Drivetrain.h"
#include "SwerveModule.h"
#include "Constants.h"
#include <frc/SerialPort.h>
#include <networktables/NetworkTable.h>

#include <rev/CANSparkMax.h>

#include "Indexer.h"
#include "Shooter.h"

#include "AutoConstants.h"

class Robot : public frc::TimedRobot {
private:
  frc::XboxController driver{0};
  frc::XboxController codriver{1};

  constants::swerveConstants::SwerveConfig config;
  Drivetrain m_swerve{config};
  bool headingControl;

  Indexer indexer;
  Shooter shooter;
  
  rev::CANSparkMax climber{23, rev::CANSparkMaxLowLevel::MotorType::kBrushless};

  double m_speedScale;

  enum AutoRoutine {
      k5Ball,  //new 7 ball without the wall pickup
      k5Ball_Old, //Old 7 Ball without the wall pickup
      k7Ball, //Starts on the outer edge of the tarmac near the center
      k7Ball_Old, //Start near center
      k2Ball, //Starts on the outer edge of the tarmac near the center
      k2Ball_Old, //Starts near center
      k1Ball, //auto from RCR Playoffs
  } m_autoSelected;

  frc::SendableChooser<AutoRoutine> m_autoChooser;
  const std::string kAuto5Ball = "5 Ball";
  const std::string kAuto5BallOld = "5 Ball Old";
  const std::string kAuto2BallOld = "2 Ball Old";
  const std::string kAuto2Ball = "2 Ball";
  const std::string kAuto1Ball = "1 Ball";
  const std::string kAuto7Ball = "7 Ball";
  const std::string kAuto7BallOld = "7 Ball Old";
  
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

  
  void Gen7Ball(){
    //set Starting Pose
    //Setup is with the front bumper on the outside tarmac with 
    //the corner in the corner nearest the ball 1 location
    frc::Pose2d x(7.58_m, 1.93_m, frc::Rotation2d(-88.5_deg));
    m_swerve.SetPose(x); //Set the robot to this pose 
    
    //init trajector selector
    trajSelect = 0;

    //create trajectory 1 - starting to ball 1
    frc::Pose2d trajStartPoint{x.X(),x.Y(),x.Rotation().Degrees()};
    frc::Pose2d Ball1{303_in, 30_in, -90_deg};
    //midpoint list between starting point and Ball 1
    std::vector<frc::Translation2d> waypoints_to_Ball1{
      frc::Translation2d{302_in, 50_in}//Setup/pickup ball 1
      };

    //slowdown region for Ball 1
    frc::RectangularRegionConstraint slowRegionBall1(frc::Translation2d{280_in, 0_in},
                                                 frc::Translation2d{340_in, 60_in},
                                                 frc::MaxVelocityConstraint{.5_mps}
                                                );
    
    frc::Pose2d Ball3{50_in,50_in,-135_deg};
    std::vector<frc::Translation2d> MidPointBall1ToBall3{
      frc::Translation2d{6.39_m, 1.8_m},
      frc::Translation2d{5.44_m, 1.9_m},
      frc::Translation2d{3_m, 1.9_m}
      };

    frc::Pose2d Ball5{35_in,110_in,-180_deg};//
    
    frc::Pose2d FarShootPoint{80_in,80_in,-161.4_deg};
    std::vector<frc::Translation2d> MidPointBall3ToShoot{
      frc::Translation2d{60_in, 60_in}
    };
    std::vector<frc::Translation2d> MidPointBall5ToShoot{
      frc::Translation2d{65_in, 110_in}
    };

    //configure traj with speed and acceleration 
    units::meters_per_second_t maxV(constants::swerveConstants::MaxSpeed*.6);
    units::meters_per_second_squared_t maxA(constants::swerveConstants::MaxAcceleration*.5);
    frc::TrajectoryConfig config{ maxV, maxA};
    
    config.AddConstraint(slowRegionBall1);//Add Slow Region

    //gen traj
    traj = frc::TrajectoryGenerator::GenerateTrajectory(trajStartPoint, 
                                                        waypoints_to_Ball1, 
                                                        Ball1, config);
    traj2 = frc::TrajectoryGenerator::GenerateTrajectory(Ball1, 
                                                        MidPointBall1ToBall3, 
                                                        Ball3 , config);
    traj3 = frc::TrajectoryGenerator::GenerateTrajectory(Ball3, 
                                                         MidPointBall3ToShoot, 
                                                          FarShootPoint , config);
    traj4 = frc::TrajectoryGenerator::GenerateTrajectory(FarShootPoint, 
                                                        MidPointBall5ToShoot, 
                                                        Ball5 , config);
    traj5 = frc::TrajectoryGenerator::GenerateTrajectory(Ball5, 
                                                        MidPointBall5ToShoot, 
                                                        FarShootPoint , config);
  }
  
  void Gen7BallOld(){
    //set Starting Pose
    frc::Pose2d x(316.15_in, 112.2_in, frc::Rotation2d(-111_deg));
    m_swerve.SetPose(x);
    
    //init trajector selector
    trajSelect = 0;

    //create trajectory 1 - starting to ball 1
    frc::Pose2d trajStartPoint{x.X(),x.Y(),x.Rotation().Degrees()};
    

    frc::Pose2d Ball2{175_in,85_in, -145.5_deg};//  208,82
    std::vector<frc::Translation2d> waypoints_to_Ball1_and_Ball2{
      frc::Translation2d{312_in, 35_in},//Setup/pickup ball 1
      frc::Translation2d{300_in, 35_in},//Pickup ball 1  //Heading = -105_deg
      frc::Translation2d{280_in, 107_in}//Setup for 2nd ball  //Heading = -145.5
      };

    //slowdown region for Ball 1
    frc::RectangularRegionConstraint slowRegionBall1(frc::Translation2d{280_in, 60_in},
                                                 frc::Translation2d{340_in, 0_in},
                                                 frc::MaxVelocityConstraint{.5_mps}
                                                );
    
    frc::Pose2d Ball3{50_in,50_in,-135_deg};
    std::vector<frc::Translation2d> MidPointBall2ToBall3{
      frc::Translation2d{150_in, 77_in}
      };

    frc::Pose2d Ball5{40_in,110_in,-180_deg};//
    std::vector<frc::Translation2d> MidPointShootToBall5{
      frc::Translation2d{40_in, 110_in}
      };
    
    frc::Pose2d FarShootPoint{80_in,80_in,-161.4_deg};
    std::vector<frc::Translation2d> MidPointBall3ToShoot{
      frc::Translation2d{60_in, 60_in}
    };
    std::vector<frc::Translation2d> MidPointBall5ToShoot{
      frc::Translation2d{80_in, 130_in}
    };

    //configure traj with speed and acceleration 
    units::meters_per_second_t maxV(constants::swerveConstants::MaxSpeed*.6);
    units::meters_per_second_squared_t maxA(constants::swerveConstants::MaxAcceleration*.5);
    frc::TrajectoryConfig config{ maxV, maxA};
    
    config.AddConstraint(slowRegionBall1);//Add Slow Region

    //gen traj
    traj = frc::TrajectoryGenerator::GenerateTrajectory(trajStartPoint, 
                                                        waypoints_to_Ball1_and_Ball2, 
                                                        Ball2, config);


    

    traj2 = frc::TrajectoryGenerator::GenerateTrajectory(Ball2, 
                                                        MidPointBall2ToBall3, 
                                                        Ball3 , config);
    traj3 = frc::TrajectoryGenerator::GenerateTrajectory(Ball3, 
                                                         MidPointBall3ToShoot, 
                                                          FarShootPoint , config);
    traj4 = frc::TrajectoryGenerator::GenerateTrajectory(FarShootPoint, 
                                                        MidPointShootToBall5, 
                                                        Ball5 , config);
    traj5 = frc::TrajectoryGenerator::GenerateTrajectory(Ball5, 
                                                        MidPointBall5ToShoot, 
                                                        FarShootPoint , config);

    
    

  }

  void Gen2Ball(){
    //set Starting Pose
    frc::Pose2d x(6.74_m, 5.68_m, frc::Rotation2d(136.5_deg));
    m_swerve.SetPose(x);
    
    //init trajector selector
    trajSelect = 0;

    //create trajectory 1 - starting to ball 1
    frc::Pose2d trajStartPoint{x.X(),x.Y(),x.Rotation().Degrees()};
    
    frc::Pose2d Ball{4.95_m,6.2_m,148.2_deg};
    std::vector<frc::Translation2d> interiorWaypoints{
      frc::Translation2d{5.9_m, 5.9_m}};
    
    //set speed scaler
    //units::scalar_t scaleSpeed = 1.0;
    //configure traj with speed and acceleration 
    units::meters_per_second_t maxV(constants::swerveConstants::MaxSpeed*.4);
    units::meters_per_second_squared_t maxA(constants::swerveConstants::MaxAcceleration*.5);
    frc::TrajectoryConfig config{ maxV, maxA};
    
    //don't care for swerve
    config.SetReversed(false);
    
    //gen traj
    traj = frc::TrajectoryGenerator::GenerateTrajectory(trajStartPoint, interiorWaypoints, Ball, config);

    frc::Pose2d Ball2{5.59_m,7.25_m,0_deg};
    std::vector<frc::Translation2d> interiorWaypoints2{
      frc::Translation2d{5.0_m, 7.25_m}};
    
    traj2 = frc::TrajectoryGenerator::GenerateTrajectory(Ball, interiorWaypoints2, Ball2, config);

  }

  void Gen2BallOld(){
    //set Starting Pose
    frc::Pose2d x(282.4_in, 191.2_in, frc::Rotation2d(159_deg));
    m_swerve.SetPose(x);
    
    //init trajector selector
    trajSelect = 0;

    //create trajectory 1 - starting to ball 1
    frc::Pose2d trajStartPoint{x.X(),x.Y(),x.Rotation().Degrees()};
    
    frc::Pose2d Ball{194_in,258_in,159_deg};
    std::vector<frc::Translation2d> interiorWaypoints{
      frc::Translation2d{241_in, 224_in}};
    
    //set speed scaler
    //units::scalar_t scaleSpeed = 1.0;
    //configure traj with speed and acceleration 
    units::meters_per_second_t maxV(constants::swerveConstants::MaxSpeed*.4);
    units::meters_per_second_squared_t maxA(constants::swerveConstants::MaxAcceleration*.5);
    frc::TrajectoryConfig config{ maxV, maxA};
    
    //don't care for swerve
    config.SetReversed(false);
    
    //gen traj
    traj = frc::TrajectoryGenerator::GenerateTrajectory(trajStartPoint, interiorWaypoints, Ball, config);

    
  }

  void Gen1Ball(){
    //set Starting Pose
    frc::Pose2d x(6.89_m, 4.48_m, frc::Rotation2d(159_deg));
    m_swerve.SetPose(x);
    
    //6.89m, 4.48m, 158 
    //3.82 4.52, 178


    //init trajector selector
    trajSelect = 0;

    //create trajectory 1 - starting to ball 1
    frc::Pose2d trajStartPoint{x.X(),x.Y(),x.Rotation().Degrees()};
    frc::Pose2d trajEndPoint{3.82_m,4.52_m, 178_deg};
    std::vector<frc::Translation2d> interiorWaypoints{
      frc::Translation2d{5.38_m, 4.61_m}};


    //set speed scaler
    units::scalar_t scaleSpeed = .5;
    //configure traj with speed and acceleration 
    units::meters_per_second_t maxV(constants::swerveConstants::MaxSpeed*scaleSpeed);
    units::meters_per_second_squared_t maxA(constants::swerveConstants::MaxAcceleration*.4);
    frc::TrajectoryConfig config{ maxV, maxA};
    
    //don't care for swerve
    config.SetReversed(false);
    
    //gen traj
    traj = frc::TrajectoryGenerator::GenerateTrajectory(trajStartPoint, interiorWaypoints, trajEndPoint, config);

    
  }

  void GenTraj(){
    AutoRoutine t = m_autoChooser.GetSelected();
    /*if(t==m_autoSelected){
      return;
    }*/

    m_autoSelected = t;
    switch(m_autoSelected){
      case AutoRoutine::k1Ball : Gen1Ball(); break;
      case AutoRoutine::k2Ball : Gen2Ball(); break;
      case AutoRoutine::k2Ball_Old : Gen2BallOld(); break;
      case AutoRoutine::k5Ball : Gen7Ball(); break;
      case AutoRoutine::k7Ball : Gen7Ball(); break;
      case AutoRoutine::k5Ball_Old : Gen7BallOld(); break;
      case AutoRoutine::k7Ball_Old : Gen7BallOld(); break;

    }
  }

  void Run7Ball(){
    frc::Pose2d p;
    frc::Pose2d rp = m_swerve.GetPose();
    units::degree_t heading;
    switch(trajSelect){
      //Shoot Low Goal
      case 0: shooter.SetShooter(constants::ShooterFront_NearLow, constants::ShooterBack_NearLow);
              indexer.Update(false);
              indexer.Fire();
              if(autoTimer.Get()>0.25_s){
                trajSelect++;
                autoTimer.Reset();
              }
              break;
      //Get Ball 1 and 2
      case 1: p = traj.Sample(autoTimer.Get()).pose; 
              if(IsTargetVisable()){ //Out of Blackout, if target visable, use it
                heading = m_swerve.GetHeading().Degrees()-GetTargetAngleDelta();
              }else{//Target Not visable, Guess!
                heading = -90_deg;
              }
              m_swerve.DrivePos(p.X(), p.Y(), heading);
              shooter.SetShooter(GetAutoFrontShooterSpeed(),GetAutoFrontShooterSpeed()*1.4);//Get shooter Up to Speed
              indexer.Update(false);
              if(autoTimer.Get()>(traj.TotalTime())){
                trajSelect++;
                autoTimer.Reset();
              }
              break;
      //Shoot ball 1&2
      case 2: //shooter.SetShooter(constants::ShooterFront_Auto, constants::ShooterBack_Auto);
              shooter.SetShooter(GetAutoFrontShooterSpeed(),GetAutoFrontShooterSpeed()*1.4);
              if(autoTimer.Get()>.5_s){ //Give the Shooter 1 sec to get to the right speed
                indexer.SetUpper(1.0);  //Fire 2 balls
                indexer.SetLower(.6);
              }
              //m_swerve.SetTargetHeading(m_swerve.GetHeading().Degrees()-GetTargetAngleDelta());
              m_swerve.DriveXY(0_mps,0_mps);
              if(/*(!indexer.GetLowerBallSensor()&&!indexer.GetLowerBallSensor())||*/autoTimer.Get()>1.5_s){
                trajSelect++;
                autoTimer.Reset();
                indexer.Update(false);
              }
              break;
      //Get Ball 2&3
      case 3: p = traj2.Sample(autoTimer.Get()).pose;
              if(p.X()>290_in ){ //Starting Region - Maintain Heading
                heading = -110_deg;
              }else{
                if(p.X()<=290_in && p.X()>130_in ){ //Approaching Ball 2 In target Blackout from Target Overhang
                  heading = -180_deg;
                }else{
                  heading = -135_deg;
                }
              }
              m_swerve.DrivePos(p.X(), p.Y(), heading);//Need to approach Straight because of wall
              indexer.Update(false);
              shooter.SetShooter(GetAutoFrontShooterSpeed(),GetAutoFrontShooterSpeed()*1.4);//Get shooter Up to Speed
              if(autoTimer.Get()>(traj2.TotalTime()+1_s)){
                trajSelect++;
                autoTimer.Reset();
              }
              break;
      //Return to Shoot
      case 4: p = traj3.Sample(autoTimer.Get()).pose;
              if(p.Y()<60_in){ //Wait to get away from wall to turn
                heading = -135_deg;
              }else{
                if(IsTargetVisable()){
                  heading = m_swerve.GetHeading().Degrees()-GetTargetAngleDelta();  
                }else{
                  heading = -161.4_deg; //Backup heading
                }
              }
              m_swerve.DrivePos(p.X(), p.Y(), heading); 
              indexer.Update(false);
              shooter.SetShooter(GetAutoFrontShooterSpeed(),GetAutoFrontShooterSpeed()*1.4);//Get shooter Up to Speed
              if(autoTimer.Get()>(traj3.TotalTime()+0_s)){
                trajSelect++;
                autoTimer.Reset();
              }
              break;
      //Shoot 3&4
      case 5: //shooter.SetShooter(constants::ShooterFront_Auto, constants::ShooterBack_Auto);
              shooter.SetShooter(GetAutoFrontShooterSpeed(),GetAutoFrontShooterSpeed()*1.4);
              if(autoTimer.Get()>1_s){
                indexer.SetUpper(1.0);
                indexer.SetLower(.6);
              }
              m_swerve.DriveXY(0_mps,0_mps);
              if(autoTimer.Get()>1_s){
                trajSelect++;
                if(m_autoSelected== k5Ball_Old){//5 ball complete, jump to final stop state
                  trajSelect = 100;
                }
                autoTimer.Reset();
              }
              break;
      //Go For Balls 5&6
      case 6: p = traj4.Sample(autoTimer.Get()).pose;
              m_swerve.DrivePos(p.X(), p.Y(), 180_deg);//approach corner wall
              indexer.Update(false);
              shooter.SetShooter(GetAutoFrontShooterSpeed(),GetAutoFrontShooterSpeed()*1.4);//Get shooter Up to Speed
              if(autoTimer.Get()>(traj4.TotalTime()+1_s)){
                trajSelect++;
                autoTimer.Reset();
              }
              break;
      //Return to Shoot
      case 7: p = traj5.Sample(autoTimer.Get()).pose;
              m_swerve.DrivePos(p.X(), p.Y(), m_swerve.GetHeading().Degrees()-GetTargetAngleDelta());
              indexer.Update(false);
              shooter.SetShooter(GetAutoFrontShooterSpeed(),GetAutoFrontShooterSpeed()*1.4);//Get shooter Up to Speed
              if(autoTimer.Get()>(traj5.TotalTime()+0_s)){
                trajSelect++;
                autoTimer.Reset();
              }
              break;
      //Shoot 5&6
      case 8: //shooter.SetShooter(constants::ShooterFront_Auto, constants::ShooterBack_Auto);
              shooter.SetShooter(GetAutoFrontShooterSpeed(),GetAutoFrontShooterSpeed()*1.4);
              if(autoTimer.Get()>1_s){
                indexer.SetUpper(1.0);
                indexer.SetLower(.6);
              }
              m_swerve.DriveXY(0_mps,0_mps);
              if(autoTimer.Get()>1_s){
                trajSelect++;
                autoTimer.Reset();
              }
              break;
      default: m_swerve.DriveXY(0_mps,0_mps);
    }
  }

  void Run7BallOld(){
    frc::Pose2d p;
    frc::Pose2d rp = m_swerve.GetPose();
    units::degree_t heading;
    switch(trajSelect){
      //Shoot Low Goal
      case 0: shooter.SetShooter(constants::ShooterFront_NearLow, constants::ShooterBack_NearLow);
              indexer.Update(false);
              indexer.Fire();
              if(autoTimer.Get()>0.25_s){
                trajSelect++;
                autoTimer.Reset();
              }
              break;
      //Get Ball 1 and 2
      case 1: p = traj.Sample(autoTimer.Get()).pose; 
              if(p.X()>290_in && p.Y()>=80_in){ //Starting Region - Maintain Heading
                heading = m_swerve.GetHeading().Degrees();
              }else{
                if(p.X()>290_in && p.Y()<80_in){ //Approaching Ball 1
                  heading = -105_deg;
                }else{
                  if(p.X()<=290_in && p.X()>230_in ){ //Approaching Ball 2 In target Blackout from Target Overhang
                    heading = -145.5_deg;
                  }else{
                    if(IsTargetVisable()){ //Out of Blackout, if target visable, use it
                      heading = m_swerve.GetHeading().Degrees()-GetTargetAngleDelta();
                    }else{//Target Not visable, Guess!
                      heading = -145.5_deg;
                    }
                  }
                }
              }
              m_swerve.DrivePos(p.X(), p.Y(), heading);
              shooter.SetShooter(GetAutoFrontShooterSpeed(),GetAutoFrontShooterSpeed()*1.4);//Get shooter Up to Speed
              indexer.Update(false);
              if(autoTimer.Get()>(traj.TotalTime())){
                trajSelect++;
                autoTimer.Reset();
              }
              break;
      //Shoot ball 1&2
      case 2: //shooter.SetShooter(constants::ShooterFront_Auto, constants::ShooterBack_Auto);
              shooter.SetShooter(GetAutoFrontShooterSpeed(),GetAutoFrontShooterSpeed()*1.4);
              if(autoTimer.Get()>1_s){ //Give the Shooter 1 sec to get to the right speed
                indexer.SetUpper(1.0);  //Fire 2 balls
                indexer.SetLower(.6);
              }
              //m_swerve.SetTargetHeading(m_swerve.GetHeading().Degrees()-GetTargetAngleDelta());
              m_swerve.DriveXY(0_mps,0_mps);
              if(/*(!indexer.GetLowerBallSensor()&&!indexer.GetLowerBallSensor())||*/autoTimer.Get()>2_s){
                trajSelect++;
                autoTimer.Reset();
                indexer.Update(false);
              }
              break;
      //Get Ball 3&4
      case 3: p = traj2.Sample(autoTimer.Get()).pose;
              m_swerve.DrivePos(p.X(), p.Y(), -135_deg);//Need to approach Straight because of wall
              indexer.Update(false);
              shooter.SetShooter(GetAutoFrontShooterSpeed(),GetAutoFrontShooterSpeed()*1.4);//Get shooter Up to Speed
              if(autoTimer.Get()>(traj2.TotalTime()+1_s)){
                trajSelect++;
                autoTimer.Reset();
              }
              break;
      //Return to Shoot
      case 4: p = traj3.Sample(autoTimer.Get()).pose;
              if(p.Y()<60_in){ //Wait to get away from wall to turn
                heading = -135_deg;
              }else{
                if(IsTargetVisable()){
                  heading = m_swerve.GetHeading().Degrees()-GetTargetAngleDelta();  
                }else{
                  heading = -161.4_deg; //Backup heading
                }
              }
              m_swerve.DrivePos(p.X(), p.Y(), heading); 
              indexer.Update(false);
              shooter.SetShooter(GetAutoFrontShooterSpeed(),GetAutoFrontShooterSpeed()*1.4);//Get shooter Up to Speed
              if(autoTimer.Get()>(traj3.TotalTime()+0_s)){
                trajSelect++;
                autoTimer.Reset();
              }
              break;
      //Shoot 3&4
      case 5: //shooter.SetShooter(constants::ShooterFront_Auto, constants::ShooterBack_Auto);
              shooter.SetShooter(GetAutoFrontShooterSpeed(),GetAutoFrontShooterSpeed()*1.4);
              if(autoTimer.Get()>1_s){
                indexer.SetUpper(1.0);
                indexer.SetLower(.6);
              }
              m_swerve.DriveXY(0_mps,0_mps);
              if(autoTimer.Get()>1_s){
                trajSelect++;
                if(m_autoSelected== k5Ball_Old){//5 ball complete, jump to final stop state
                  trajSelect = 100;
                }
                autoTimer.Reset();
              }
              break;
      //Go For Balls 5&6
      case 6: p = traj4.Sample(autoTimer.Get()).pose;
              m_swerve.DrivePos(p.X(), p.Y(), 180_deg);//approach corner wall
              indexer.Update(false);
              shooter.SetShooter(GetAutoFrontShooterSpeed(),GetAutoFrontShooterSpeed()*1.4);//Get shooter Up to Speed
              if(autoTimer.Get()>(traj4.TotalTime()+1_s)){
                trajSelect++;
                autoTimer.Reset();
              }
              break;
      //Return to Shoot
      case 7: p = traj5.Sample(autoTimer.Get()).pose;
              m_swerve.DrivePos(p.X(), p.Y(), m_swerve.GetHeading().Degrees()-GetTargetAngleDelta());
              indexer.Update(false);
              shooter.SetShooter(GetAutoFrontShooterSpeed(),GetAutoFrontShooterSpeed()*1.4);//Get shooter Up to Speed
              if(autoTimer.Get()>(traj5.TotalTime()+0_s)){
                trajSelect++;
                autoTimer.Reset();
              }
              break;
      //Shoot 5&6
      case 8: //shooter.SetShooter(constants::ShooterFront_Auto, constants::ShooterBack_Auto);
              shooter.SetShooter(GetAutoFrontShooterSpeed(),GetAutoFrontShooterSpeed()*1.4);
              if(autoTimer.Get()>1_s){
                indexer.SetUpper(1.0);
                indexer.SetLower(.6);
              }
              m_swerve.DriveXY(0_mps,0_mps);
              if(autoTimer.Get()>1_s){
                trajSelect++;
                autoTimer.Reset();
              }
              break;
      default: m_swerve.DriveXY(0_mps,0_mps);
    }
  }

  void Run2Ball(){
    frc::Pose2d p;
    units::degree_t heading;
    switch(trajSelect){
      case 0: p = traj.Sample(autoTimer.Get()).pose; 
              shooter.SetShooter(GetAutoFrontShooterSpeed(),GetAutoFrontShooterSpeed()*1.4);
              indexer.SetFeederSpeed(1.0);
              if(m_swerve.GetPose().X()>5.65_m){
                heading = 165_deg;
              }else{
                if(IsTargetVisable()){
                  heading = m_swerve.GetHeading().Degrees()-GetTargetAngleDelta();
                }else{
                  heading = 145_deg;
                }
              }
              m_swerve.DrivePos(p.X(), p.Y(), heading);
              if(autoTimer.Get()>(traj.TotalTime()+0.5_s)){
                trajSelect++;
                autoTimer.Reset();
              }
              break;
      case 1: //shooter.SetShooter(constants::ShooterFront_Auto, constants::ShooterBack_Auto);
              shooter.SetShooter(GetAutoFrontShooterSpeed(),GetAutoFrontShooterSpeed()*1.4);
              if(autoTimer.Get()>0.5_s){
                indexer.SetUpper(1.0);
                indexer.SetLower(.6);
              }
              m_swerve.DriveXY(0_mps,0_mps);
              if(autoTimer.Get()>2.5_s){
                trajSelect++;
                autoTimer.Reset();
              }
              break;
      case 2: p = traj2.Sample(autoTimer.Get()).pose; 
              shooter.SetShooter(constants::ShooterFront_NearLow,constants::ShooterBack_NearLow);
              indexer.SetFeederSpeed(1.0);
              m_swerve.DrivePos(p.X(), p.Y(), 0_deg);
              if(autoTimer.Get()>(traj.TotalTime())){
                trajSelect++;
                autoTimer.Reset();
              }
              break;
      case 3: if(indexer.Ready2Fire()){
                indexer.Fire();
                trajSelect++;
                autoTimer.Reset();
              }
              break;
      case 4: if(indexer.IsUpperEmpty()){
                shooter.SetShooter(constants::ShooterFront_NearHigh,constants::ShooterBack_NearHigh);
                trajSelect++;
              }
              break;
      default: m_swerve.DriveXY(0_mps,0_mps);
               m_swerve.SetTargetHeading(m_swerve.GetHeading().Degrees());
    }
  }

  void Run2BallOld(){
    frc::Pose2d p;
    units::degree_t heading;
    switch(trajSelect){
      case 0: p = traj.Sample(autoTimer.Get()).pose; 
              shooter.SetShooter(GetAutoFrontShooterSpeed(),GetAutoFrontShooterSpeed()*1.4);
              indexer.SetFeederSpeed(1.0);
              if(autoTimer.Get()>(traj.TotalTime()-.5_s)){
                heading = m_swerve.GetHeading().Degrees();
              }else{
                if(IsTargetVisable()){
                  heading = m_swerve.GetHeading().Degrees()-GetTargetAngleDelta();
                }else{
                  heading = 148.2_deg;
                }
              }
              m_swerve.DrivePos(p.X(), p.Y(), heading);
              if(autoTimer.Get()>(traj.TotalTime()+2_s)){
                trajSelect++;
                autoTimer.Reset();
              }
              break;
      case 1: //shooter.SetShooter(constants::ShooterFront_Auto, constants::ShooterBack_Auto);
              shooter.SetShooter(GetAutoFrontShooterSpeed(),GetAutoFrontShooterSpeed()*1.4);
              if(autoTimer.Get()>1_s){
                indexer.SetUpper(1.0);
                indexer.SetLower(.6);
              }
              m_swerve.DriveXY(0_mps,0_mps);
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
    indexer.SetFeederPos(false);
    switch(trajSelect){
      case 0: if(autoTimer.Get()>3_s){
                trajSelect++;
                autoTimer.Reset();
              }
              break;
      case 1: p = traj.Sample(autoTimer.Get()).pose; 
              indexer.SetFeederPos(false);
              shooter.SetShooter(GetAutoFrontShooterSpeed(),GetAutoFrontShooterSpeed()*1.4);
              if(autoTimer.Get()>2_s){
                m_swerve.DrivePos(p.X(), p.Y(), m_swerve.GetHeading().Degrees());
              }else{
                m_swerve.DrivePos(p.X(), p.Y(), m_swerve.GetHeading().Degrees()-GetTargetAngleDelta());
              }
              if(autoTimer.Get()>(traj.TotalTime()+1_s)){
                trajSelect++;
                autoTimer.Reset();
              }
              break;
      case 2: //shooter.SetShooter(constants::ShooterFront_Auto, constants::ShooterBack_Auto);
          shooter.SetShooter(GetAutoFrontShooterSpeed(),GetAutoFrontShooterSpeed()*1.4);
          indexer.Fire();
          m_swerve.DriveXY(0_mps,0_mps);
          if(autoTimer.Get()>2_s){
            trajSelect++;
            autoTimer.Reset();
          }
          break;
      default: m_swerve.DriveXY(0_mps,0_mps);
               m_swerve.SetTargetHeading(m_swerve.GetHeading().Degrees()-GetTargetAngleDelta()/1.0);
    }
  }

  units::inch_t GetEstimatedRangeToTarget(){
    frc::Pose2d rp = m_swerve.GetPose();
    units::inch_t x = constants::field::TargetX - rp.X();
    units::inch_t y = constants::field::TargetY - rp.Y();
    return units::math::hypot(x,y);
  }
  
  units::degree_t GetEstimatedAngleToTarget(){
    frc::Pose2d rp = m_swerve.GetPose();
    units::inch_t x = constants::field::TargetX - rp.X();
    units::inch_t y = constants::field::TargetY - rp.Y();
    return frc::Rotation2d( units::math::atan2(y,x)).Degrees();
  }

 public:
  void RobotInit() override {
    //frc::SmartDashboard::PutData("Drivetrain", &m_swerve);
    //m_swerve.PutChildSendables();
    
    m_autoChooser.SetDefaultOption(kAuto7Ball, AutoRoutine::k7Ball);
    m_autoChooser.AddOption(kAuto5Ball, AutoRoutine::k5Ball);
    m_autoChooser.AddOption(kAuto2Ball, AutoRoutine::k2Ball);
    m_autoChooser.AddOption(kAuto1Ball, AutoRoutine::k1Ball);
    m_autoChooser.AddOption(kAuto7BallOld, AutoRoutine::k7Ball_Old);
    m_autoChooser.AddOption(kAuto5BallOld, AutoRoutine::k5Ball_Old);
    m_autoChooser.AddOption(kAuto2BallOld, AutoRoutine::k2Ball_Old);
    frc::SmartDashboard::PutData("Auto Modes", &m_autoChooser);

   
    m_swerve.SetHeading(180_deg);
    headingControl =  true;
    manualShooterMode = false;
    driveMode = DriveMode::HeadingControl;

    m_swerve.ResetDriveEncoders();    

    //AddPeriodic([&] { m_swerve.UpdateOdometry(); }, constants::swerveConstants::DriveCANBusPeriod, 5_ms);

   
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
    if(IsTargetVisable()){
      double targetOffsetAngle_Vertical = limelight->GetNumber("ty", 0.0);
      double targetOffsetAngle_VerticalLen = limelight->GetNumber("tvert", 0.0)/2.0;
      targetOffsetAngle_Vertical = targetOffsetAngle_Vertical+targetOffsetAngle_VerticalLen;
      // how many degrees back is your limelight rotated from perfectly vertical?
      double limelightMountAngleDegrees = 38.0;

      // distance from the center of the Limelight lens to the floor
      double limelightHeightInches = 20.0;

      // distance from the target to the floor
      double goalHeightInches = 104.0;

      double angleToGoalDegrees = limelightMountAngleDegrees + targetOffsetAngle_Vertical;
      double angleToGoalRadians = angleToGoalDegrees * (3.14159 / 180.0);

      //calculate distance
      double distanceFromLimelightToGoalInches = (goalHeightInches - limelightHeightInches)/tan(angleToGoalRadians);


      return units::inch_t(distanceFromLimelightToGoalInches);
    }else{
      return GetEstimatedRangeToTarget();
    }
    //*/return 0_in;
  }


  void RobotPeriodic() override {
    
    m_swerve.UpdateOdometry();
    
    m_swerve.SendData();
    //indexer.SendData();
    shooter.SendData();

    frc::SmartDashboard::PutNumber("Pose To Target Angle", GetEstimatedAngleToTarget().value());
    frc::SmartDashboard::PutNumber("Pose To Target Range", GetEstimatedRangeToTarget().value());

    frc::SmartDashboard::PutNumber("Vision Target Distance (inch)", GetTargetRange().value());
    frc::SmartDashboard::PutNumber("Vision Target Angle (deg)", GetTargetAngleDelta().value());
    frc::SmartDashboard::PutBoolean("Vision Target Visable", IsTargetVisable());
    
    
  }

  void DisabledPeriodic() override {
    //GenTraj();
  }

  void AutonomousInit() override {
    GenTraj();// make sure no last minute routine change
 
    indexer.SetFeederSpeed(.7);
    indexer.SetFeederPos(true);
 
    autoTimer.Reset();
    autoTimer.Start();
    trajSelect = 0; 
  }

  void AutonomousPeriodic() override {
    //DriveWithJoystick(false);
    m_swerve.UpdateOdometry();
    //indexer.Update(false);
    switch(m_autoSelected){
      case AutoRoutine::k1Ball : Run1Ball(); break;
      case AutoRoutine::k2Ball : Run2Ball(); break;
      case AutoRoutine::k2Ball_Old : Run2BallOld(); break;
      case AutoRoutine::k5Ball : Run7Ball(); break;
      case AutoRoutine::k7Ball : Run7Ball(); break;
      case AutoRoutine::k5Ball_Old : Run7BallOld(); break;
      case AutoRoutine::k7Ball_Old : Run7BallOld(); break;
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
    manualShooterMode = true;
    
  }

  void Climber(){
    int i = driver.GetPOV();
    switch(i){
      //If Climber is too fast in either direction, change here.
      case 0 : climber.Set(1.0); break;//Full Speed Up
      case 180 : climber.Set(-1.0);break;//Full Speed Down
      default : climber.Set(0.0);break;
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
      double minFeed = .35;
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
  }

  inline units::feet_per_second_t GetAutoFrontShooterSpeed(){
    units::inch_t d;
    if(IsTargetVisable()){
      d = GetTargetRange();
    }else{
      d = GetEstimatedRangeToTarget();
    }
    return ((GetTargetRange()-constants::ShooterNearDistance)/1_in)*.072_fps +constants::ShooterFront_NearHigh;//was 0.066
  }

  void Shoot(){
    //Codriver right & left Bumper send fire command
    if(codriver.GetRightBumperPressed()){
      indexer.Fire();
    }

    double backRoller = -frc::ApplyDeadband(codriver.GetRightY(), .5);
    double frontRoller = -frc::ApplyDeadband(codriver.GetLeftY(), .5);
    //see if codriver is trying to manually control shooter
    if(fabs(backRoller) >.1 || fabs(frontRoller)> .1){
      manualShooterMode = true;
    }

    //If in manual mode, Drive shooter manually
    if(manualShooterMode){
      //units::feet_per_second_t autoSpeed = ((GetTargetRange()-constants::ShooterNearDistance)/1_in)*.066_fps +constants::ShooterFront_NearHigh;
      shooter.SetShooter(GetAutoFrontShooterSpeed(),GetAutoFrontShooterSpeed()*1.4);
      
      
    }

    //Near High Goal Shot
    if(codriver.GetAButtonPressed()){
      //exit velocity of 21fps, 75deg, 1.25ft from target edge
      shooter.SetShooter(constants::ShooterFront_NearHigh, constants::ShooterBack_NearHigh);//@20deg 27, 37 is solid
      manualShooterMode = false;
    }
    //Near Low Goal Shot
    if(codriver.GetXButtonPressed()){
      shooter.SetShooter(constants::ShooterFront_NearLow, constants::ShooterBack_NearLow);//@20deg 20, 15 is solid
      manualShooterMode = false;
    }
    //Mid High Goal Shot
    if(codriver.GetBButtonPressed()){
      shooter.SetShooter(constants::ShooterFront_MidHigh, constants::ShooterBack_MidHigh);
      manualShooterMode = false;
    }
    //Auto High Goal Shot
    if(codriver.GetYButtonPressed()){
      //units::feet_per_second_t autoSpeed = ((GetTargetRange()-constants::ShooterNearDistance)/1_in)*.066_fps +constants::ShooterFront_NearHigh;
      //shooter.SetShooter(autoSpeed,autoSpeed*1.4);
      shooter.SetShooter(constants::ShooterFront_FarHigh, constants::ShooterBack_FarHigh);
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
    Climber();
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
