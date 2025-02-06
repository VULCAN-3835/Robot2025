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
import frc.robot.Constants.alageaSubsystemConstants;

public class AlageaSubsystem extends SubsystemBase {
  /** Creates a new AlageaIntakeSubsystem. */
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
  private final Config config;
  private final SysIdRoutine sysID;

  public AlageaSubsystem() {
    this.angleMotor = new TalonFX(alageaSubsystemConstants.angleMotorID);
    this.powerMotor = new TalonFX(alageaSubsystemConstants.powerMotorID);

    this.ballDetector = new AnalogInput(alageaSubsystemConstants.ballDetectorID);
    this.lowLimitSwitch = new DigitalInput(alageaSubsystemConstants.limitSwitchID);
    this.angleEncoder = new DutyCycleEncoder(alageaSubsystemConstants.angleEncoderID);

    this.pidController = new PIDController(alageaSubsystemConstants.kP, alageaSubsystemConstants.kI, alageaSubsystemConstants.kD);
    pidController.setTolerance(alageaSubsystemConstants.pidTolerence);

    this.constraints = new Constraints(maxVelocity, maxAcceleration);
    this.profiledPIDController = new ProfiledPIDController(alageaSubsystemConstants.profiledkP, alageaSubsystemConstants.profiledkI, alageaSubsystemConstants.profiledkD, constraints);

    this.hasBallTimer = new Timer();
    angleMotor.setNeutralMode(NeutralModeValue.Brake);

    //0.05
    this.config = new Config( Volts.of(0.02).per(Millisecond), Volts.of(1.5), Seconds.of(2));
    this.sysID = new SysIdRoutine(config,
     new SysIdRoutine.Mechanism(this::setVoltage,
    Log->{
      angleMotor.get();
      angleMotor.getMotorVoltage();
      angleMotor.getVelocity();
    } ,
     this));
  }

  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return this.sysID.quasistatic(direction);
  }
  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return this.sysID.dynamic(direction);
  }

  // checks if the system detects the ball
  public boolean hasBall() {
    return ballDetector.getVoltage() > alageaSubsystemConstants.ballDetectorThreshold;

  }
  public void setVoltage(Voltage volts){
    angleMotor.setVoltage(volts.in(Volts));
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

    if (targetDegrees >= alageaSubsystemConstants.minAngle.in(Degrees)
        && targetDegrees <= alageaSubsystemConstants.maxAngle.in(Degrees)) {
      pidController.setSetpoint(targetAngle.in(Degrees));
    }

    pidController.setSetpoint(targetDegrees);
    profiledPIDController.setGoal(targetDegrees);
  }

  // sets the robot in the predefined resting angle
  public void setRestAngle() {
    setAngle(alageaSubsystemConstants.restAngle);

  }

  // sets the robot in the predefined collecting angle
  public void setCollectAngle() {
    setAngle(alageaSubsystemConstants.collectAngle);

  }

  public void setHoldAngle() {
    // sets the robot in the predefined holding angle
    setAngle(alageaSubsystemConstants.holdAngle);

  }

  public void setShootingAngle() {
    setAngle(alageaSubsystemConstants.scoreAngle);
  }

  public boolean isSystemAtShootingAngle() {
    double currentAngle = getAngle().in(Degrees);
    double targetAngle = alageaSubsystemConstants.scoreAngle.in(Degrees);
    return Math.abs(currentAngle -  targetAngle) <= pidController.getPositionTolerance();
  }

  public boolean isSystemAtCollectingAngle() {
    double currentAngle = getAngle().in(Degrees);
    double targetAngle = alageaSubsystemConstants.scoreAngle.in(Degrees);
    return Math.abs(currentAngle - targetAngle) <= pidController.getPositionTolerance();
  }

  public void setPower(double power) {
    powerMotor.set(power);
  }

  public Command waitForCollectionCommand() {
    return new WaitUntilCommand(() -> hasBallTimer.get() > alageaSubsystemConstants.collectTime);
  }

  public void collectingAlgea() {
    setPower(alageaSubsystemConstants.collectingPower);
  }

  public void shootAlagea() {
    setPower(alageaSubsystemConstants.shootingPower);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    //calculates the ouput of the motor 
    double pidOutput = pidController.calculate(getAngle().in(Degrees));
    double profiledPIDOutput = pidController.calculate(getAngle().in(Degrees));

    double power = pidOutput + profiledPIDOutput;

    SmartDashboard.putNumber("algea intake encoder", getAngle().in(Degrees));
    SmartDashboard.putBoolean("algea intake limit Switch", getLowLimitSwitch());
    SmartDashboard.putNumber("algea intake encoder", getAngle().in(Degrees));
    SmartDashboard.putNumber("piece detector value", ballDetector.getVoltage());
    SmartDashboard.putNumber("PID output", power);


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
    if (powerMotor.get() < 0 && hasBallTimer.get() > alageaSubsystemConstants.collectTime) {
      setPower(0);
    }
  }
}
