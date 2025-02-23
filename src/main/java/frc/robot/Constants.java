// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Centimeter;
import static edu.wpi.first.units.Units.Centimeters;
import static edu.wpi.first.units.Units.Rotation;



import com.ctre.phoenix6.configs.Slot0Configs;

import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.RobotConfig;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.DistanceUnit;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.MutPer;
import edu.wpi.first.units.measure.Per;
import frc.robot.Util.ElevatorStates;

public final class Constants {

  public static class OperatorConstants {
    public static final int driverControllerPort = 0;
    public static final int buttonControllerPort = 1;
    public static final double kDeadband = 0.1;

  }

  public static class EndAccessoryConstants {

    public static final int angleMotorID = 60;
    public static final int powerMotorID = 61;

    public static final double maxVelocity = 70;
    public static final double maxAcceleration = 100;


    public static final int angleEncoderID = 5;
    public static final int pieceDetectorID = 1;

    public static final double kMotorPowerL1 = -0.25;
    public static final double kMotorPowerL2 = 0.4;
    public static final double kMotorPowerL3 = 0.4;
    public static final double kMotorPowerL4 = 0.5;
    public static final double kMotorPowerIntake = 0.4;

    public static final double removeAlgeaPower = 0.5;

    public static final double kHasPieceVoltageThreshold = 1;

    public static final Angle kMaxAngle = Degrees.of(191);
    public static final Angle kMinAngle = Degrees.of(50);

    public static final Angle targetDropAngleL1 = Degrees.of(66);
    public static final Angle targetDropAngleL2 = Degrees.of(162);
    public static final Angle targetDropAngleL3 = Degrees.of(162);
    public static final Angle targetDropAngleL4 = Degrees.of(191);

    public static final Angle targetRemoveAlgea = Degrees.of(85);//169
 
    public static final Angle targetAngleRest = Degrees.of(54);

    public static final Angle targetIntakeAngle = Degrees.of(85);

    public static final double ProfiledkP = 0.005;
    public static final double profiledkI = 0;
    public static final double profiledkD = 0;

    public static final Angle armAngleTolerence = Degrees.of(3);

  }

  public static class algeaSubsystemConstants{

    public static final int angleMotorID = 41;
    public static final int powerMotorID = 40;

    public static final int ballDetectorID = 0;
    public static final int limitSwitchID = 3;
    public static final int angleEncoderID = 1;

    public static final double ballDetectorThreshold =1.8;

    public static final Angle minAngle = Degrees.of(103);
    public static final Angle maxAngle = Degrees.of(150);
    public static final Angle restAngle = Degrees.of(103);
    public static final Angle collectAngle = Degrees.of(150);
    public static final Angle holdAngle = Degrees.of(105);
    public static final Angle scoreAngle = Degrees.of(150);

    public static final Angle pidTolerence = Degrees.of(5); 

    public static final double collectingPower = 0.5;
    public static final double shootingPower = -0.6;
    public static final double holdPower = 0.1;

    public static final double maxVelocity = 10;
    public static final double maxAcceleration = 12;

    public static final double profiledkP = 0.008;
    public static final double profiledkI = 0;
    public static final double profiledkD = 0;


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
        public static final int kLeftFrontDriveID = 12; // CAN ID
        public static final int kRightFrontDriveID = 10; // CAN ID
        public static final int kLeftBackDriveID = 13; // CAN ID
        public static final int kRightBackDriveID =11 ; // CAN ID
        // Ports for angle motors
        public static final int kLeftFrontSteerID = 22; // CAN ID
        public static final int kRightFrontSteerID =20 ; // CAN ID
        public static final int kLeftBackSteerID = 23; // CAN ID
        public static final int kRightBackSteerID = 21; // CAN ID
        // Ports for encoders
        public static final int kLeftFrontEncID = 32; // CAN ID
        public static final int kRightFrontEncID =30 ; // CAN ID
        public static final int kLeftBackEncID =33 ; // CAN ID
        public static final int kRightBackEncID = 31; // CAN ID

    // Offsets for absolute encoders in rotations (i.e: 360 degrees = 1 rotation):
    // public static final double kLeftFrontOffset = -0.029296875;
    // public static final double kRightFrontOffset = -0.4111328125;
    // public static final double kLeftBackOffset = 0.406494140625;
    // public static final double kRightBackOffset = -0.228515625;

    public static final double kLeftFrontOffset = -1.4658203125;
    public static final double kRightFrontOffset = -0.510986328125;
    public static final double kLeftBackOffset = 0.04296875;
    public static final double kRightBackOffset = -0.66259765625;
    // Which motors are inverted: public static final boolean frontLeftDriveInverted
    // = true;
    public static final boolean kLeftFrontInverted = true;
    public static final boolean kRightFrontInverted = true;
    public static final boolean kLeftBackInverted = true;
    public static final boolean kRightBackInverted = true;

    public static final double kMaxDrivingVelocity = 3.5;
    public static final double kTeleDriveMaxAccelerationUnitsPerSec = 5;
    public static final double kTeleDriveMaxSpeedMetersPerSec = 3.5;
    public static final double kTeleDriveMaxAngulerSpeedRadiansPerSec = Math.PI * 1.5;

    // Distance between centers of right and left wheels on robot meters
    public static final double kTrackWidth = 0.6357;
    // Distance between front and back wheels on robot meters
    public static final double kWheelBase = 0.6357;
    // Distance between middle of robot to module wheel
    public static final double kWheelRadius = 0.38205;

    // the mass of the robot in KG
    public static final double kMassKG = 51;
    // the moment of inertia of the robot
    public static final double kMOI = 6.81;

  public static class distanceConstants{

    //TODO: find the right distance
    public static final Distance source = Meters.of(0.1);

    //TODO: fill the reefs value
    public static final Distance topReefDistance = Centimeter.of(0);
    public static final Distance topRightReefDistance = Centimeter.of(0);
    public static final Distance topLeftReefDistance = Centimeter.of(0);

    public static final Distance bottomReefDistance = Centimeter.of(0);
    public static final Distance bottomRightReefDistance = Centimeter.of(0);
    public static final Distance bottomLeftReefDistance = Centimeter.of(0);

    public static final Distance proccesorDistance = Centimeter.of(0);


  }

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
  

  public static class ElevatorConstant {


    public static final Distance coralL1 = Centimeter.of(0);
    public static final Distance coralL2 = Centimeter.of(0);
    public static final Distance coralL3 = Centimeter.of(20.52);
    public static final Distance coralL4 = Centimeter.of(41);
    public static final Distance restDistance = Centimeter.of(0);
    public static final Distance sourceDistance = Centimeter.of(15);
    public static final Distance removeAlgea = Centimeter.of(22);

    public static final double pidTolerence = 1.5;

    public static final int elevatorMotorID = 50;
    public static final int limitSwitchID = 7;

    public static final double kS = 0;
    public static final double kG = 0;
    public static final double kV = 0;

    public static final double ProfiledkP = 0.16;
    public static final double ProfiledkI = 0;
    public static final double ProfiledkD = 0;

    public static final double maxVelocity = 18;
    public static final double maxAcceleration = 20;



    public static double restPower = 0;

    // We'll need to know how much we get for every rotation, and use that number
    // here:
    public static Per<DistanceUnit, AngleUnit> distancePerRotation = Centimeters.of(1).div(Rotation.of(3.2));

    public static Distance enumDistance(ElevatorStates elevatorStates) {
      switch (elevatorStates) {
        case coralL1:
          System.out.println("L1");
          return coralL1;
        case coralL2:
        System.out.println("L2");
          return coralL2;
        case coralL3:
        System.out.println("L3");
          return coralL3;
        case coralL4:
        System.out.println("L4");
          return coralL4;
        case source:
        System.out.println("source");
          return sourceDistance;
        case rest:
        System.out.println("rest");
          return restDistance;
        case removeAlgea:
          return removeAlgea;
      }
      return null;
    }

  }

  public static class ClimbSubsystemConstants {
    //TODO: change values to actual values

    public static final int climbMotorPort = 55; 
    public static final double climbMotorPower = 0.2;

    public static final int limitSwitchPort = 0;
    public static final double motorRatio = 125;
    // the degrees of the arm closed
    public static final Angle degreesForOpen = Degrees.of(0);
    public static final Angle degreesForClose = Degrees.of(0);
    public static final double closeClimbMotorPower = 0;
  }
}
