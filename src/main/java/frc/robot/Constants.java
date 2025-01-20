// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.RobotConfig;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;

public final class Constants {

  public static class OperatorConstants {
    public static final int driverController = 0;
    public static final int commandController = 0;
    public static final double kDeadband =0.1 ;
  }
  public static class alageaIntakeSubsystemConstants{
    public static final int angleMotorID = 0;
    public static final int PowerMotorID = 1;
    public static final int angleEncoderID = 2;
    public static final int ballDetectorID = 3;
    public static final int limitSwitchID = 4;
    public static final int initAngle = 90;
    public static final double restAngle = 0;
    public static final double holdAngle = 0;
    public static final double collectingAngle = 0;
    public static final double maxAngle = 180;
    public static final double minAngle = 90;
    public static final double ballDetectorThreshold = 0;

        
  }

  public static class OVCameraUtilConstants {
    public static final double kValidArea =10;
    
    //camera height from the ground in meters
    public static final double kCameraHeight = 0;//TODO: change value to actual value

    //camera degrees from upper view(should be zero if the camera is straight)
    public static final double kCameraDegrees = 0;

    //camera pitch in degrees
    public static final double kCameraPitch = 0;//TODO: change value to actual value

    public static final int kCameraAprilTagPipeLine = 0;//TODO: change value to actual value
    public static final double kCameraXAcordingToRobot = 0;//TODO: change value to actual value in inches
    public static final double kCameraYAcordingToRobot = 0;//TODO: change value to actual value in inches

  
  }

  public static class ModuleConstants {
    public static final double kWheelDiameterMeters = Units.inchesToMeters(4); // Module wheel diameter in meters
    public static final double kWheelCircumference = kWheelDiameterMeters * Math.PI;
    public static final double kDriveMotorGearRatio = 6.75; // Module drive motor gear ratio
    public static final double kSteerMotorGearRatio = 12.8; // Module steer motor gear ratio
    
    public static double kFeedforwardGainSteer = 0.11; // The feed forward gain for the module steer control

    public static Slot0Configs getSteerMotorGains() { 
      Slot0Configs kSteerMotorGains = new Slot0Configs();
      kSteerMotorGains.withKP(30); // The proportional gain for the module steer control
      return kSteerMotorGains;
    }
    
    public static Slot0Configs getDriveMotorGains() { 
      Slot0Configs kSteerMotorGains = new Slot0Configs();
      kSteerMotorGains.withKP(0.15); // The proportional gain for the module steer control 
      return kSteerMotorGains;
    }

    public static SimpleMotorFeedforward leftFrontFF = new SimpleMotorFeedforward(0.21599, 2.2476, 0.040257);
    public static SimpleMotorFeedforward leftBackFF = new SimpleMotorFeedforward(0.20676, 2.1653, 0.16537); // previous constants - (0.20676, 2.1653, 0.16537)
    public static SimpleMotorFeedforward rightFrontFF = new SimpleMotorFeedforward(0.1788, 2.257, 0.036611);
    public static SimpleMotorFeedforward rightBackFF = new SimpleMotorFeedforward(0.11961, 2.3274, 0.13714);

    public static double kModuleAngleDeadband = 0.001;

    /* Swerve Current Limiting */
    public static final int kSteerCurrentLimit = 25;
    public static final int kSteerCurrentThreshold = 30;
    public static final double kSteerCurrentThresholdTime = 0.1;
    public static final boolean kSteerEnableCurrentLimit = true;

    public static final int kDriveCurrentLimit = 30;
    public static final int kDriveCurrentThreshold = 40;
    public static final double kDriveCurrentThresholdTime = 0.1;
    public static final boolean kDriveEnableCurrentLimit = true;
  }
  public static class ChassisConstants { 
    // Ports for driving motors
    public static final int kLeftFrontDriveID = 11; // CAN ID
    public static final int kRightFrontDriveID = 13; // CAN ID
    public static final int kLeftBackDriveID = 12; // CAN ID
    public static final int kRightBackDriveID = 10; // CAN ID
    // Ports for angle motors
    public static final int kLeftFrontSteerID = 21; // CAN ID
    public static final int kRightFrontSteerID = 23; // CAN ID
    public static final int kLeftBackSteerID = 22; // CAN ID
    public static final int kRightBackSteerID = 20; // CAN ID
    // Ports for encoders 
    public static final int kLeftFrontEncID = 31; // CAN ID
    public static final int kRightFrontEncID = 33; // CAN ID
    public static final int kLeftBackEncID = 32; // CAN ID
    public static final int kRightBackEncID = 30; // CAN ID
    // Offsets for absolute encoders in rotations (i.e: 360 degrees = 1 rotation):
//    public static final double kLeftFrontOffset = -0.029296875;
//    public static final double kRightFrontOffset = -0.4111328125;
//    public static final double kLeftBackOffset = 0.406494140625;
//    public static final double kRightBackOffset = -0.228515625;
    public static final double kLeftFrontOffset = -0.160888671875;
    public static final double kRightFrontOffset = -0.423583984375;
    public static final double kLeftBackOffset = -0.216064453125;
    public static final double kRightBackOffset = 0.259765625;
    // Which motors are inverted:                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                   public static final boolean frontLeftDriveInverted = true;
    public static final boolean kLeftFrontInverted = true; 
    public static final boolean kRightFrontInverted = true;
    public static final boolean kLeftBackInverted = true;
    public static final boolean kRightBackInverted = true;

    public static final double kMaxDrivingVelocity = 1.5;
    public static final double kTeleDriveMaxAccelerationUnitsPerSec = 5;
    public static final double kTeleDriveMaxSpeedMetersPerSec = 1.5;
    public static final double kTeleDriveMaxAngulerSpeedRadiansPerSec = Math.PI*1.5;

    // Distance between centers of right and left wheels on robot meters
    public static final double kTrackWidth = 0.5403;
    // Distance between front and back wheels on robot meters
    public static final double kWheelBase = 0.5403;
    // Distance between middle of robot to module wheel
    public static final double kWheelRadius = 0.38205;

    // the mass of the robot in KG
    public static final double kMassKG = 0;
    // the moment of inertia of the robot
    public static final double kMOI = 0;



    // Swerve Kinematics:
    public static final SwerveDriveKinematics kDriveKinematics =
      new SwerveDriveKinematics(
        new Translation2d(kWheelBase / 2, kTrackWidth / 2), //Left front
        new Translation2d(kWheelBase / 2, -kTrackWidth / 2), //Right front
        new Translation2d(-kWheelBase / 2, kTrackWidth / 2), //Left back
        new Translation2d(-kWheelBase / 2, -kTrackWidth / 2) //Right back
      );
    public static final RobotConfig DEFAUL_ROBOT_CONFIG = new RobotConfig(kMassKG, kMOI, 
    new ModuleConfig(kWheelRadius,
      kMaxDrivingVelocity,
      kWheelBase, DCMotor.getKrakenX60(4), 
      ModuleConstants.kDriveCurrentLimit, 1 ),
      kDriveKinematics.getModules());  

      public static RobotConfig getConfig(){
        RobotConfig config;
        try{
          config = RobotConfig.fromGUISettings();
        } catch (Exception e){
          e.printStackTrace();
          config = DEFAUL_ROBOT_CONFIG;
        }
        return config;

      }
      
  }
  
}

