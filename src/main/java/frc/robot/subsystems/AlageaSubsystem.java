// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class AlageaSubsystem extends SubsystemBase {
  /** Creates a new AlageaIntakeSubsystem. */
  private final TalonFX angleMotor;
  private final TalonFX powerMotor;
  private final StatusSignal<Angle> getPosition;
  private final AnalogInput ballDetector;
  private PIDController pidController;
  private final DigitalInput limitSwitch;
  private Angle currentSetpoint;

  public AlageaSubsystem() {
    angleMotor = new TalonFX(Constants.alageaSubsystemConstants.angleMotorID);
    powerMotor = new TalonFX(Constants.alageaSubsystemConstants.PowerMotorID);
    ballDetector = new AnalogInput(Constants.alageaSubsystemConstants.ballDetectorID);
    limitSwitch = new DigitalInput(Constants.alageaSubsystemConstants.limitSwitchID);
    this.getPosition = this.angleMotor.getPosition();

    pidController = new PIDController(0, 0, 0);
    pidController.setTolerance(Constants.alageaSubsystemConstants.pidTolerence);
    currentSetpoint = Constants.alageaSubsystemConstants.restAngle;

  }

  public boolean hasBall() {
    // checks if the system detects the ball

    return ballDetector.getVoltage() > Constants.alageaSubsystemConstants.ballDetectorThreshold;

  }

  public Angle getAngle() {
    // gets the current angle
    return getPosition.getValue();

  }

  private void setAngle(Angle targetAngle) {
    // sets the desired robot angle
    double targetDegrees = targetAngle.in(Degrees);
    if (targetDegrees >= Constants.alageaSubsystemConstants.minAngle.in(Degrees)
        && targetDegrees <= Constants.alageaSubsystemConstants.maxAngle.in(Degrees)) {
      currentSetpoint = targetAngle;
      pidController.setSetpoint(targetDegrees);
    } else {
      System.out.println("Invalid angle, please enter a valid angle");
    }
  }

  public void setRestAngle() {
    // sets the robot in the predefined resting angle
    setAngle(Constants.alageaSubsystemConstants.restAngle);

  }

  public void setCollectAngle() {
    // sets the robot in the predefined collecting angle
    setAngle(Constants.alageaSubsystemConstants.collectingAngle);

  }

  public void setHoldAngle() {
    // sets the robot in the predefined holding angle
    setAngle(Constants.alageaSubsystemConstants.holdAngle);

  }

  public void setShootingAngle() {
    setAngle(Constants.alageaSubsystemConstants.shootingAngle);
  }

  public boolean isSystemAtShootingAngle() {
    double currentAngle = getAngle().in(Degrees);
    double targetAngle = Constants.alageaSubsystemConstants.shootingAngle.in(Degrees);
    return Math.abs(currentAngle - targetAngle) <= pidController.getPositionTolerance();
  }

  public boolean isSystemAtCollectingAngle() {
    double currentAngle = getAngle().in(Degrees);
    double targetAngle = Constants.alageaSubsystemConstants.collectingAngle.in(Degrees);
    return Math.abs(currentAngle - targetAngle) <= pidController.getPositionTolerance();
  }

  

  public void setPower(double power) {
    // checkes if the system is trying to go past the limitSwitch and if so, stops
    // the motor
    if (limitSwitch.get() && power < 0) {
      powerMotor.set(0);
    } else {
      powerMotor.set(power);
    }
  }

  public void setCollectingPower() {
    setPower(Constants.alageaSubsystemConstants.collectingPower);
  }

  public void shootAlagea() {
    setPower(Constants.alageaSubsystemConstants.shootingPower);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // checks if limitSwitch is true and if so, sets the angle to the intalizing
    // position
    // also constantly checkes and updates the desired settings for the motor
    
    if (limitSwitch.get()) {
      angleMotor.set(Constants.alageaSubsystemConstants.initAngle.in(Degrees));
    }
    setPower((pidController.calculate(getAngle().in(Degrees))));
  }
}