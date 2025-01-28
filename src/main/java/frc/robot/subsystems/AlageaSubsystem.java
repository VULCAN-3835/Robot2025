// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Rotations;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
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

  public AlageaSubsystem() {
    this.angleMotor = new TalonFX(alageaSubsystemConstants.angleMotorID);
    this.powerMotor = new TalonFX(alageaSubsystemConstants.powerMotorID);

    this.ballDetector = new AnalogInput(alageaSubsystemConstants.ballDetectorID);
    this.lowLimitSwitch = new DigitalInput(alageaSubsystemConstants.limitSwitchID);
    this.angleEncoder = new DutyCycleEncoder(alageaSubsystemConstants.angleEncoderID);

    pidController = new PIDController(0, 0, 0);
    pidController.setTolerance(alageaSubsystemConstants.pidTolerence);

    this.hasBallTimer = new Timer();
  }

  // checks if the system detects the ball
  public boolean hasBall() {
    return ballDetector.getVoltage() > alageaSubsystemConstants.ballDetectorThreshold;

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
  }

  // sets the robot in the predefined resting angle
  public void setRestAngle() {
    setAngle(alageaSubsystemConstants.restAngle);

  }

  // sets the robot in the predefined collecting angle
  public void setCollectAngle() {
    setAngle(alageaSubsystemConstants.collectingAngle);

  }

  public void setHoldAngle() {
    // sets the robot in the predefined holding angle
    setAngle(alageaSubsystemConstants.holdAngle);

  }

  public void setShootingAngle() {
    setAngle(alageaSubsystemConstants.shootingAngle);
  }

  public boolean isSystemAtShootingAngle() {
    double currentAngle = getAngle().in(Degrees);
    double targetAngle = alageaSubsystemConstants.shootingAngle.in(Degrees);
    return Math.abs(currentAngle -  targetAngle) <= pidController.getPositionTolerance();
  }

  public boolean isSystemAtCollectingAngle() {
    double currentAngle = getAngle().in(Degrees);
    double targetAngle = alageaSubsystemConstants.collectingAngle.in(Degrees);
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

    double voltage = pidController.calculate(getAngle().in(Degrees));
    if (getLowLimitSwitch() && voltage > 0) {
      angleMotor.set(0);
    } else {
      angleMotor.set(voltage);
    }

    if (hasBall()) {
      if (hasBallTimer.get() == 0) {
        hasBallTimer.start();
      }
    } else {
      hasBallTimer.stop();
      hasBallTimer.reset();
    }
    if (powerMotor.get() < 0 && hasBallTimer.get() > alageaSubsystemConstants.collectTime) {
      setPower(0);
    }
  }
}