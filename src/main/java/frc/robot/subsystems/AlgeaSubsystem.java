// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Millisecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volt;
import static edu.wpi.first.units.Units.Volts;
import static edu.wpi.first.units.Units.VoltsPerMeterPerSecond;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;
import frc.robot.Constants.algeaSubsystemConstants;

public class AlgeaSubsystem extends SubsystemBase {
  /** Creates a new AlgeaIntakeSubsystem. */
  private final TalonFX angleMotor;
  private final TalonFX powerMotor;
  private final AnalogInput ballDetector;
  private final PIDController pidController;
  private final DigitalInput lowLimitSwitch;
  private final DutyCycleEncoder angleEncoder;
  private final Timer hasBallTimer;
  private final Constraints constraints;
  private final ProfiledPIDController profiledPIDController;

  private final double maxVelocity = 0;
  private final double maxAcceleration = 0;

  //SysID objects
  private final Config config;
  private final SysIdRoutine sysID;

  public AlgeaSubsystem() {
    this.angleMotor = new TalonFX(algeaSubsystemConstants.angleMotorID);
    this.powerMotor = new TalonFX(algeaSubsystemConstants.powerMotorID);

    this.ballDetector = new AnalogInput(algeaSubsystemConstants.ballDetectorID);
    this.lowLimitSwitch = new DigitalInput(algeaSubsystemConstants.limitSwitchID);
    this.angleEncoder = new DutyCycleEncoder(algeaSubsystemConstants.angleEncoderID);

    this.pidController = new PIDController(algeaSubsystemConstants.kP, algeaSubsystemConstants.kI, algeaSubsystemConstants.kD);
    pidController.setTolerance(algeaSubsystemConstants.pidTolerence.in(Degrees));

    this.constraints = new Constraints(maxVelocity, maxAcceleration);
    this.profiledPIDController = new ProfiledPIDController(algeaSubsystemConstants.profiledkP, algeaSubsystemConstants.profiledkI, algeaSubsystemConstants.profiledkD, constraints);
    profiledPIDController.setTolerance(algeaSubsystemConstants.pidTolerence.in(Degrees));

    this.hasBallTimer = new Timer();
    angleMotor.setNeutralMode(NeutralModeValue.Brake);

    angleEncoder.setDutyCycleRange(algeaSubsystemConstants.minAngle.in(Degrees), algeaSubsystemConstants.maxAngle.in(Degrees));

    //the SysID configs, for explenation go to elevator subsystem in lines 77-79
    this.config = new Config( Volts.of(0.01).per(Millisecond), Volts.of(0.5), Seconds.of(2));
    this.sysID = new SysIdRoutine(config,
     new SysIdRoutine.Mechanism(this::setVoltage,
    Log->{
      angleMotor.get();
      angleMotor.getMotorVoltage();
      angleMotor.getVelocity();
    } ,
     this));
  }

  //SysID Quasistatics tests commands
  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return this.sysID.quasistatic(direction);
  }

  //SysID dynamic tests commands
  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return this.sysID.dynamic(direction);
  }


  public void setVoltage(Voltage volts){
    angleMotor.setVoltage(volts.in(Volts));
  }

  // checks if the system detects the ball
  public boolean hasBall() {
    return ballDetector.getVoltage() > algeaSubsystemConstants.ballDetectorThreshold;
  }


  // gets the current angle
  public Angle getAngle() {
    return Rotations.of(angleEncoder.get());

  }

  public boolean getLowLimitSwitch() {
    return lowLimitSwitch.get();
  }

  // sets the desired robot angle
  private void setAngle(Angle targetAngle) {
    double targetDegrees = targetAngle.in(Degrees);

    if (targetDegrees >= algeaSubsystemConstants.minAngle.in(Degrees)
        && targetDegrees <= algeaSubsystemConstants.maxAngle.in(Degrees)) {
      pidController.setSetpoint(targetAngle.in(Degrees));
    }

    pidController.setSetpoint(targetDegrees);
    profiledPIDController.setGoal(targetDegrees);
  }

  // sets the robot in the resting angle
  public void setRestAngle() {
    setAngle(algeaSubsystemConstants.restAngle);

  }

  // sets the robot in the collecting angle
  public void setCollectAngle() {
    setAngle(algeaSubsystemConstants.collectAngle);

  }
   
  // sets the robot in the holding angle
  public void setHoldAngle() {
    setAngle(algeaSubsystemConstants.holdAngle);

  }

  public void setShootingAngle() {
    setAngle(algeaSubsystemConstants.scoreAngle);
  }

  public boolean isSystemAtShootingAngle() {
    double currentAngle = getAngle().in(Degrees);
    double targetAngle = algeaSubsystemConstants.scoreAngle.in(Degrees);
    return Math.abs(currentAngle -  targetAngle) <= pidController.getErrorTolerance();
  }

  public boolean isSystemAtCollectingAngle() {
    double currentAngle = getAngle().in(Degrees);
    double targetAngle = algeaSubsystemConstants.scoreAngle.in(Degrees);
    return Math.abs(currentAngle - targetAngle) <= pidController.getErrorTolerance();
  }

  public void setPower(double power) {
    powerMotor.set(power);
  }

  public Command waitForCollectionCommand() {
    return new WaitUntilCommand(() -> hasBallTimer.get() > algeaSubsystemConstants.collectTime);
  }

  public void collectingAlgea() {
    setPower(algeaSubsystemConstants.collectingPower);
  }

  public void shootAlgea() {
    setPower(algeaSubsystemConstants.shootingPower);
  }

  // algea SysID to use this paste it in the configureXboxBinding method in robot container
  // start with reverse direction becuase gears are flipped
  // xboxControllerDrive.a().whileTrue(alageaSubsystem.sysIdDynamic(SysIdRoutine.Direction.kForward));
  // xboxControllerDrive.b().whileTrue(alageaSubsystem.sysIdDynamic(SysIdRoutine.Direction.kReverse));
  // xboxControllerDrive.y().whileTrue(alageaSubsystem.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
  // xboxControllerDrive.x().whileTrue(alageaSubsystem.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    //calculates the ouput of the motor 
    double pidOutput = pidController.calculate(getAngle().in(Degrees));
    double profiledPIDOutput = pidController.calculate(getAngle().in(Degrees));

    double power = pidOutput + profiledPIDOutput;

    SmartDashboard.putNumber("AlgeaSubsystem/algea intake encoder", getAngle().in(Degrees));
    SmartDashboard.putBoolean("AlgeaSubsystem/algea intake limit Switch", getLowLimitSwitch());
    SmartDashboard.putNumber("AlgeaSubsystem/piece detector value", ballDetector.getVoltage());
    SmartDashboard.putBoolean("AlgeaSubsystem/has ball?", hasBall());


    if (getLowLimitSwitch() && power > 0) {
      angleMotor.set(0);
    } else {
      angleMotor.set(power);
    }

    if (hasBall()) {
      powerMotor.setNeutralMode(NeutralModeValue.Brake);
      if (hasBallTimer.get() == 0) {
        hasBallTimer.start();
      }
    } else {
      powerMotor.setNeutralMode(NeutralModeValue.Coast);
      hasBallTimer.stop();
      hasBallTimer.reset();
    }
    if (powerMotor.get() < 0 && hasBallTimer.get() > algeaSubsystemConstants.collectTime) {
      setPower(0);
    }
  }
}
