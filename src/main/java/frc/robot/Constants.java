// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Centimeter;
import static edu.wpi.first.units.Units.Centimeters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Rotation;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.StatusSignal;
import java.util.Map;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.System_StateValue;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.DistanceUnit;
import edu.wpi.first.units.Unit;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.MutPer;
import edu.wpi.first.units.measure.Per;
import frc.robot.Util.ElevatorStates;

public final class Constants {

  public static class OperatorConstants {
    public static final int driverController = 0;
    public static final int commandController = 0;
    public static final double kDeadband = 0.1;
  }
  public static class alageaSubsystemConstants{
    public static final int angleMotorID = 0;
    public static final int PowerMotorID = 1;
    public static final int angleEncoderID = 2;
    public static final int ballDetectorID = 3;
    public static final int limitSwitchID = 4;
    public static final Angle initAngle = Degrees.of(90);
    public static final Angle restAngle = Degrees.of(0);
    public static final Angle holdAngle = Degrees.of(0);
    public static final Angle collectingAngle = Degrees.of(0);;
    public static final Angle maxAngle = Degrees.of(180);;
    public static final Angle minAngle = Degrees.of(90);;
    public static final double ballDetectorThreshold = 0;
    public static final double collectingPower = -0.5;
    public static final double shootingPower = 0.5;
    public static final Angle shootingAngle = Degrees.of(90);;
    public static final double pidTolerence =  0.5;
    public static final double restingPower = 0;


        
  }

  public static class OVCameraUtilConstants {
    public static final double kValidArea = 10;

    // camera height from the ground in meters
    public static final double kCameraHeight = 0;// TODO: change value to actual value

    // camera degrees from upper view(should be zero if the camera is straight)
    public static final double kCameraDegrees = 0;

    // camera pitch in degrees
    public static final double kCameraPitch = 0;// TODO: change value to actual value

    public static final int kCameraAprilTagPipeLine = 0;// TODO: change value to actual value
    public static final double kCameraXAcordingToRobot = 0;// TODO: change value to actual value in inches
    public static final double kCameraYAcordingToRobot = 0;// TODO: change value to actual value in inches

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
    public static SimpleMotorFeedforward leftBackFF = new SimpleMotorFeedforward(0.20676, 2.1653, 0.16537); // previous
                                                                                                            // constants
                                                                                                            // -
                                                                                                            // (0.20676,
                                                                                                            // 2.1653,
                                                                                                            // 0.16537)
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
    // public static final double kLeftFrontOffset = -0.029296875;
    // public static final double kRightFrontOffset = -0.4111328125;
    // public static final double kLeftBackOffset = 0.406494140625;
    // public static final double kRightBackOffset = -0.228515625;
    public static final double kLeftFrontOffset = -0.160888671875;
    public static final double kRightFrontOffset = -0.423583984375;
    public static final double kLeftBackOffset = -0.216064453125;
    public static final double kRightBackOffset = 0.259765625;
    // Which motors are inverted: public static final boolean frontLeftDriveInverted
    // = true;
    public static final boolean kLeftFrontInverted = true;
    public static final boolean kRightFrontInverted = true;
    public static final boolean kLeftBackInverted = true;
    public static final boolean kRightBackInverted = true;

    public static final double kMaxDrivingVelocity = 1.5;
    public static final double kTeleDriveMaxAccelerationUnitsPerSec = 5;
    public static final double kTeleDriveMaxSpeedMetersPerSec = 1.5;
    public static final double kTeleDriveMaxAngulerSpeedRadiansPerSec = Math.PI * 1.5;

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
    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
        new Translation2d(kWheelBase / 2, kTrackWidth / 2), // Left front
        new Translation2d(kWheelBase / 2, -kTrackWidth / 2), // Right front
        new Translation2d(-kWheelBase / 2, kTrackWidth / 2), // Left back
        new Translation2d(-kWheelBase / 2, -kTrackWidth / 2) // Right back
    );
    public static final RobotConfig DEFAUL_ROBOT_CONFIG = new RobotConfig(kMassKG, kMOI,
        new ModuleConfig(kWheelRadius,
            kMaxDrivingVelocity,
            kWheelBase, DCMotor.getKrakenX60(4),
            ModuleConstants.kDriveCurrentLimit, 1),
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
  
  public static class ClimbConstants {
    TalonFX climbMotor = new TalonFX(10);
    DigitalInput ClimbLimitSwitch = new DigitalInput(12);
    public static final int climbMotorPort = 0;  
    public static final int LimitSwitchPort = 0;
  }

  public static class ElevatorConstant {

    public static Distance coralL1 = Centimeter.of(1); // I will add the real measurments later on
    public static Distance coralL2 = Centimeter.of(6);
    public static Distance coralL3 = Centimeter.of(6);
    public static Distance coralL4 = Centimeter.of(6);
    public static Distance restDistance = Centimeter.of(0);
    public static Distance sourceDistance = Centimeter.of(0);

    public static int motorLeftID = 0;//TODO: needs to change the values of the ports to actual values
    public static int motorRightID = 0;
    public static int limitSwitchID = 0;

    public static double kP = 0;
    public static double kI = 0;
    public static double kD = 0;
    public static double restPower = 0;

    // We'll need to know how much we get for every rotation, and use that number
    // here:
    public static Per<DistanceUnit, AngleUnit> distancePerRotation = Centimeters.of(20).div(Rotation.of(1));

    public static Distance enumDistance(ElevatorStates elevatorStates) {
      switch (elevatorStates) {
        case coralL1:
          return coralL1;
        case coralL2:
          return coralL2;
        case coralL3:
          return coralL3;
        case coralL4:
          return coralL4;
        case source:
          return sourceDistance;
        case rest:
          return restDistance;

      }
      return null;
    }

  }

  public static class ClimbSubsystemConstants {
    //TODO: change values to actual values

    public static final int climbMotorPort = 0; 
    public static final int limitSwitchPort = 0;
    public static final int climbMotorPower = 0;
    public static final double motorRatio =0;
    // the degrees of the arm closed
    public static final Angle degreesForOpen = Degrees.of(0);

    public static final double closeClimbMotorPower = 0;
  }
}
