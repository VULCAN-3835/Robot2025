// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.util.PathPlannerLogging;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.Constants.ChassisConstants;
import frc.robot.Constants.ModuleConstants;
import frc.robot.Util.OVCameraUtil;
import frc.robot.Util.SwerveModule;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Volts;

public class ChassisSubsystem extends SubsystemBase {
  // An enum with the names of the wheel modules
  public enum Wheels {
    LEFT_FRONT, RIGHT_FRONT, RIGHT_BACK, LEFT_BACK
  }

  // An array of the four swerve Modules
  private SwerveModule[] swerve_modules = new SwerveModule[4];

  // An array of the four swerve module's positions
  private SwerveModulePosition[] swerve_positions = new SwerveModulePosition[4];

  // Inertial Measurement unit 
  private AHRS imu;

  // Pose estimator responsible for keeping the robot's position on the field using gyro, encoders and camera detection
  private SwerveDrivePoseEstimator poseEstimator;

  private Pose2d startingPos;

  // Field object for presenting position relative to field
  private Field2d field;

  // The states of the modules
  private SwerveModuleState[] swerveModuleStates = new SwerveModuleState[] {
          new SwerveModuleState(0,Rotation2d.fromDegrees(0)),
          new SwerveModuleState(0,Rotation2d.fromDegrees(0)),
          new SwerveModuleState(0,Rotation2d.fromDegrees(0)),
          new SwerveModuleState(0,Rotation2d.fromDegrees(0))
  };

  // april tag camera and field initilazation
  private OVCameraUtil atCam;;
  private Field2d atField;


  // Sysid Rotinue
  SysIdRoutine routine;

  public ChassisSubsystem() {
    // Modules Initilization:
    this.swerve_modules[Wheels.LEFT_FRONT.ordinal()] = new SwerveModule(
      Constants.ChassisConstants.kLeftFrontDriveID,
      Constants.ChassisConstants.kLeftFrontSteerID, 
      Constants.ChassisConstants.kLeftFrontEncID,
      Constants.ChassisConstants.kLeftFrontInverted, 
      Constants.ChassisConstants.kLeftFrontOffset,
      ModuleConstants.leftFrontFF);

    this.swerve_modules[Wheels.RIGHT_FRONT.ordinal()] = new SwerveModule(
      Constants.ChassisConstants.kRightFrontDriveID,
      Constants.ChassisConstants.kRightFrontSteerID, 
      Constants.ChassisConstants.kRightFrontEncID,
      Constants.ChassisConstants.kRightFrontInverted, 
      Constants.ChassisConstants.kRightFrontOffset,
      ModuleConstants.rightFrontFF);

    this.swerve_modules[Wheels.LEFT_BACK.ordinal()] = new SwerveModule(
      Constants.ChassisConstants.kLeftBackDriveID, 
      Constants.ChassisConstants.kLeftBackSteerID,
      Constants.ChassisConstants.kLeftBackEncID,
      Constants.ChassisConstants.kLeftBackInverted,
      Constants.ChassisConstants.kLeftBackOffset,
      ModuleConstants.leftBackFF);

    this.swerve_modules[Wheels.RIGHT_BACK.ordinal()] = new SwerveModule(
      Constants.ChassisConstants.kRightBackDriveID,
      Constants.ChassisConstants.kRightBackSteerID,
      Constants.ChassisConstants.kRightBackEncID,
      Constants.ChassisConstants.kRightBackInverted,
      Constants.ChassisConstants.kRightBackOffset,
      ModuleConstants.rightBackFF);

    // Imu initlization
    this.imu = new AHRS(NavXComType.kMXP_SPI);
    
    // Field initlization
    field = new Field2d();

    // Update swerve position and heading at build
    updateSwervePositions();
    zeroHeading();

    atCam = new OVCameraUtil("limelight-front",ChassisConstants.CAMERA_POSE3D);

    // Robot starting position for odometry
    startingPos = new Pose2d(1.3, 5.52, Rotation2d.fromDegrees(0));
    
    // Initilizing a pose estimator
    this.poseEstimator = new SwerveDrivePoseEstimator(ChassisConstants.kDriveKinematics,
      getRotation2d().unaryMinus(),
      this.swerve_positions,
      startingPos);

    // Configuring the controller for the path planner
    AutoBuilder.configure(
      this::getPose, 
      this::resetOdometry, 
      () -> ChassisConstants.kDriveKinematics.toChassisSpeeds(getModStates()), 
      this::runVelc,
      new PPHolonomicDriveController(
        new PIDConstants(1.75, 0, 0), // Translation PID
        new PIDConstants(5.0, 0, 0) // Rotation PID 
      ),
      ChassisConstants.getConfig(),
      () -> !(Robot.allianceColor == "BLUE"), 
      this);

    // Set up custom logging to add the current path to a field 2d widget
    PathPlannerLogging.setLogActivePathCallback((poses) -> field.getObject("path").setPoses(poses));
    SmartDashboard.putData(field);

    atField = new Field2d();
    SmartDashboard.putData("Field", atField);

    routine = new SysIdRoutine(
      new SysIdRoutine.Config(),
      new SysIdRoutine.Mechanism(this::setSysidVolt, 
      log -> {
        // set states for all 4 modules
        for (Wheels wheel : Wheels.values()) {
        log.motor(wheel.toString())
            .voltage(swerve_modules[wheel.ordinal()].getVoltage())
            .linearPosition(Distance.ofBaseUnits(swerve_modules[wheel.ordinal()].getPosition().distanceMeters, Meters))
            .linearVelocity(LinearVelocity.ofBaseUnits(swerve_modules[wheel.ordinal()].getVelocity(), MetersPerSecond)); 
        } 
      }
      ,this));
  }

  /**
     * Updates the swerve_positions array based on the current SwerveModulePositions reported by the SwerveModules
  */
  private void updateSwervePositions() {
    this.swerve_positions[Wheels.LEFT_FRONT.ordinal()] = this.swerve_modules[Wheels.LEFT_FRONT.ordinal()].getPosition();
    this.swerve_positions[Wheels.RIGHT_FRONT.ordinal()] = this.swerve_modules[Wheels.RIGHT_FRONT.ordinal()].getPosition();
    this.swerve_positions[Wheels.LEFT_BACK.ordinal()] = this.swerve_modules[Wheels.LEFT_BACK.ordinal()].getPosition();
    this.swerve_positions[Wheels.RIGHT_BACK.ordinal()] = this.swerve_modules[Wheels.RIGHT_BACK.ordinal()].getPosition();
  }

  /**
     * Resets the heading of the robot
  */
  public void zeroHeading() {
    this.imu.reset();
  }

  /**
     * Returns the heading of the robot
     * @return The heading of the robot in degrees
  */
  public double getHeading() {
    return this.imu.getAngle();
  }

  /**
     * Returns the heading of the robot
     * @return The heading of the robot in Rotation2d
  */
  public Rotation2d getRotation2d() {
    return Rotation2d.fromDegrees(this.imu.getAngle());
  }

  /**
     * Sets the module's state to given one
     *
     * @param xVelocity  The velocity on the x axis
     * @param yVelocity  The velocity on the y axis
     * @param rot  The rotational velocity
     * @param fieldRelative  Is field relative or not
  */
  public void drive(double xVelocity, double yVelocity, double rot, boolean fieldRelative) {

    SmartDashboard.putNumber("Target X Velocity", xVelocity);
    SmartDashboard.putNumber("Target Y Velocity", yVelocity);
    SmartDashboard.putNumber("Target rot Velocity", rot);

    var invert = 1;
    var alliance = DriverStation.getAlliance();
    if (alliance.isPresent() && alliance.get() == Alliance.Red && fieldRelative)
      invert = -1;
    // Kinematics turns the Chassis speeds to desired swerveModule states depending on if field relative or not
    this.swerveModuleStates =
    Constants.ChassisConstants.kDriveKinematics.toSwerveModuleStates(
            fieldRelative? ChassisSpeeds.fromFieldRelativeSpeeds(xVelocity*invert, yVelocity*invert, rot, this.imu.getRotation2d())
                : new ChassisSpeeds(xVelocity, yVelocity, rot));

  }

  /**
     * Sets the module's state to given one
     *
     * @param chassisSpeeds The desired chassisSpeeds object for module velocities
     * @param fieldRelative  Is field relative or not
  */
  public void drive(ChassisSpeeds chassisSpeeds, boolean fieldRelative) {
    // Makes a swerve module-state array from chassisSpeeds
      this.swerveModuleStates = 
      Constants.ChassisConstants.kDriveKinematics.toSwerveModuleStates(
        fieldRelative?ChassisSpeeds.fromFieldRelativeSpeeds(chassisSpeeds, this.imu.getRotation2d())
        :chassisSpeeds
    );
  }

  /**
   * Runs the robot following trajectory
   *
   * @param speeds The desired chassisSpeeds object for module velocities
  */
  public void runVelc(ChassisSpeeds speeds) {
    ChassisSpeeds discSpeeds = ChassisSpeeds.discretize(speeds, 0.02);

    this.swerveModuleStates = ChassisConstants.kDriveKinematics.toSwerveModuleStates(discSpeeds);
  }

  /**
    * Sets the desired states of the modules to given ones
    *
    * @param desiredStates  Desired array of 4 module states
  */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    // Sets max acceleration and velocity to Wheels
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.ChassisConstants.kMaxDrivingVelocity);
    
    // Uses the set method of the SwerveModule to declare desired state of the module
    this.swerve_modules[Wheels.LEFT_FRONT.ordinal()].set(desiredStates[0]);
    this.swerve_modules[Wheels.RIGHT_FRONT.ordinal()].set(desiredStates[1]);
    this.swerve_modules[Wheels.LEFT_BACK.ordinal()].set(desiredStates[2]);
    this.swerve_modules[Wheels.RIGHT_BACK.ordinal()].set(desiredStates[3]);
  }
  
  /**
     * Command to execute quasistatic routine based on direction
     * @param direction The direction of the robot during routine
     * @return The command that executes the routine
  */
  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return this.routine.quasistatic(direction);
  }

  /**
     * Command to execute Dynamic routine based on direction
     * @param direction The direction of the robot during routine
     * @return The command that executes the routine
  */
  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return this.routine.dynamic(direction);
  }

  /**
     * Setter for swerve modules voltage, used only in sysid routine
     * @param volts The voltage to apply
  */
  private void setSysidVolt(Voltage volts) {
    double voltageDouble = volts.magnitude();
    setModulesVoltage(voltageDouble);
  }

  /**
     * Setter for swerve module voltage
     * @param volt The voltage to apply
  */
  public void setModulesVoltage(double volt) {
    this.swerve_modules[Wheels.LEFT_FRONT.ordinal()].setMotorVoltage(volt);
    this.swerve_modules[Wheels.RIGHT_FRONT.ordinal()].setMotorVoltage(volt);
    this.swerve_modules[Wheels.LEFT_BACK.ordinal()].setMotorVoltage(volt);
    this.swerve_modules[Wheels.RIGHT_BACK.ordinal()].setMotorVoltage(volt);
  }

  /**
     * Getter for the Limelight utility used by the subsystem for vision processing
     * @return Limelight utility object used by the subsystem for vision processing
  */
  public OVCameraUtil getCamera() {
    return this.atCam;
  }

  /**
     * Resets the odometry to a given pose
     * @param pose The new pose2d of the robot
  */
  private void resetOdometry(Pose2d pose) {
    this.poseEstimator.resetPosition(getRotation2d().unaryMinus(), getModPositions(), pose);
  }

  /**
     * Getter for the current pose2d estimated by the PoseEstimator
     * @return The current reported position of the robot
  */
  public Pose2d getPose() {
    return this.poseEstimator.getEstimatedPosition();
  }

  /**
     * Getter for the current reported swerve module positions
     * @return The current reported swerve module positions
  */
  public SwerveModulePosition[] getModPositions() {
    return this.swerve_positions;
  }

  /**
     * Getter for the current reported swerve module states
     * @return The current reported swerve module states
  */
  public SwerveModuleState[] getModStates() {
    return this.swerveModuleStates;
  }

  /**
     * Update pose estimator using vision data from the limelight
  */
  private void updatePoseEstimatorWithVisionBotPose() {
    Pose2d visionBotPose = this.atCam.getPoseFromCamera();
    if (visionBotPose.getX() == 0.0) {
      return;
    }

    // distance from current pose to vision estimated pose
    double poseDifference = poseEstimator.getEstimatedPosition().getTranslation().getDistance(visionBotPose.getTranslation());

    if (this.atCam.hasTarget()) {
      double xyStds;
      double degStds;

      if (this.atCam.hasTarget()) {
        xyStds = 0.5;
        degStds = 6;
      }
      else {
        return;
      }

      poseEstimator.setVisionMeasurementStdDevs(
          VecBuilder.fill(xyStds, xyStds, Units.degreesToRadians(degStds)));
      poseEstimator.addVisionMeasurement(visionBotPose,
          Timer.getFPGATimestamp() - (this.atCam.getCameraTimeStampSec()));
    }
  }

  @Override
  public void periodic() {
    setModuleStates(this.swerveModuleStates);
    
    updateSwervePositions();
    this.poseEstimator.update(getRotation2d().unaryMinus(), this.swerve_positions);
    updatePoseEstimatorWithVisionBotPose();

    if (this.atCam.hasTarget()) {
      this.poseEstimator.addVisionMeasurement(this.atCam.getPoseFromCamera(), Timer.getFPGATimestamp() - (this.atCam.getCameraTimeStampSec()));
    }
    this.field.setRobotPose(this.poseEstimator.getEstimatedPosition());
    this.atField.setRobotPose(this.atCam.getPoseFromCamera());

    SmartDashboard.putNumber("Gyro Heading", getHeading());

    SmartDashboard.putNumber("Left Front Distance",this.swerve_modules[Wheels.LEFT_FRONT.ordinal()].getPosition().distanceMeters);
    SmartDashboard.putNumber("Left Back Distance",this.swerve_modules[Wheels.LEFT_BACK.ordinal()].getPosition().distanceMeters);
    SmartDashboard.putNumber("Right Front Distance",this.swerve_modules[Wheels.RIGHT_FRONT.ordinal()].getPosition().distanceMeters);
    SmartDashboard.putNumber("Right Back Distance",this.swerve_modules[Wheels.RIGHT_BACK.ordinal()].getPosition().distanceMeters);

    SmartDashboard.putNumber("Left Front Rotation",this.swerve_modules[Wheels.LEFT_FRONT.ordinal()].getPosition().angle.getRotations());
    SmartDashboard.putNumber("Left Back Rotation",this.swerve_modules[Wheels.LEFT_BACK.ordinal()].getPosition().angle.getRotations());
    SmartDashboard.putNumber("Right Front Rotation",this.swerve_modules[Wheels.RIGHT_FRONT.ordinal()].getPosition().angle.getRotations());
    SmartDashboard.putNumber("Right Back Rotation",this.swerve_modules[Wheels.RIGHT_BACK.ordinal()].getPosition().angle.getRotations());

    SmartDashboard.putNumber("Left Front Rotation Error",this.swerve_modules[Wheels.LEFT_FRONT.ordinal()].getModuleAngleError());
    SmartDashboard.putNumber("Left Back Rotation Error",this.swerve_modules[Wheels.LEFT_BACK.ordinal()].getModuleAngleError());
    SmartDashboard.putNumber("Right Front Rotation Error",this.swerve_modules[Wheels.RIGHT_FRONT.ordinal()].getModuleAngleError());
    SmartDashboard.putNumber("Right Back Rotation Error",this.swerve_modules[Wheels.RIGHT_BACK.ordinal()].getModuleAngleError());

    SmartDashboard.putNumber("Left Front Rotation Output",this.swerve_modules[Wheels.LEFT_FRONT.ordinal()].getModuleClosedLoopOutput());
    SmartDashboard.putNumber("Left Back Rotation Output",this.swerve_modules[Wheels.LEFT_BACK.ordinal()].getModuleClosedLoopOutput());
    SmartDashboard.putNumber("Right Front Rotation Output",this.swerve_modules[Wheels.RIGHT_FRONT.ordinal()].getModuleClosedLoopOutput());
    SmartDashboard.putNumber("Right Back Rotation Output",this.swerve_modules[Wheels.RIGHT_BACK.ordinal()].getModuleClosedLoopOutput());
  
    SmartDashboard.putNumber("Left Front Drive Velocity", swerve_modules[Wheels.LEFT_FRONT.ordinal()].getVelocity());
    SmartDashboard.putNumber("Left Back Drive Velocity", swerve_modules[Wheels.LEFT_BACK.ordinal()].getVelocity());
    SmartDashboard.putNumber("Right Front Drive Velocity", swerve_modules[Wheels.RIGHT_FRONT.ordinal()].getVelocity());
    SmartDashboard.putNumber("Right Back Drive Velocity", swerve_modules[Wheels.RIGHT_BACK.ordinal()].getVelocity());

    SmartDashboard.putNumber("Left Front Drive Output", swerve_modules[Wheels.LEFT_FRONT.ordinal()].getModuleDriveOutput());
    SmartDashboard.putNumber("Left Back Drive Output", swerve_modules[Wheels.LEFT_BACK.ordinal()].getModuleDriveOutput());
    SmartDashboard.putNumber("Right Front Drive Output", swerve_modules[Wheels.RIGHT_FRONT.ordinal()].getModuleDriveOutput());
    SmartDashboard.putNumber("Right Back Drive Output", swerve_modules[Wheels.RIGHT_BACK.ordinal()].getModuleDriveOutput());

    // SmartDashboard.putNumber("tid", this.atCam.getAprilTagID());
  }
}
