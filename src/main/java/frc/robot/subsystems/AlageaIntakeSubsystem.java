// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class AlageaIntakeSubsystem extends SubsystemBase {
  /** Creates a new AlageaIntakeSubsystem. */
  private final TalonFX angleMotor;
  private final TalonFX powerMotor;
  private final StatusSignal<Angle> getPosition;
  private final AnalogInput ballDetector;
  private PIDController pidController;
  private final DigitalInput limitSwitch;

  public AlageaIntakeSubsystem() {
    angleMotor = new TalonFX(Constants.alageaIntakeSubsystemConstants.angleMotorID);
    powerMotor = new TalonFX(Constants.alageaIntakeSubsystemConstants.PowerMotorID);
    ballDetector = new AnalogInput(Constants.alageaIntakeSubsystemConstants.ballDetectorID);
    limitSwitch = new DigitalInput(Constants.alageaIntakeSubsystemConstants.limitSwitchID);
    this.getPosition = this.angleMotor.getPosition();

    pidController = new PIDController(0, 0, 0);
    pidController.setTolerance(0.5);

  }

  public boolean hasBall() {

    return ballDetector.getVoltage() > Constants.alageaIntakeSubsystemConstants.ballDetectorThreshold;

  }

  public double getAngle() {
    return getPosition.getValueAsDouble();

  }

  private void setAngle(double targetAngle) {
    if (targetAngle >= Constants.alageaIntakeSubsystemConstants.minAngle && targetAngle <= Constants.alageaIntakeSubsystemConstants.maxAngle) {
      pidController.setSetpoint(targetAngle); 
    }
  }

  public void setRestAngle() {
    setAngle(Constants.alageaIntakeSubsystemConstants.restAngle);

  }

  public void setCollectAngle() {
    setAngle(Constants.alageaIntakeSubsystemConstants.collectingAngle);

  }

  public void setHoldAngle() {
    setAngle(Constants.alageaIntakeSubsystemConstants.holdAngle);

  }

  public void setPower(double power) {
    if (limitSwitch.get() && pidController.calculate(getAngle()) < 0){
      powerMotor.set(0);
   } else {
    powerMotor.set(power);
   }

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if(limitSwitch.get()){
      powerMotor.set(Constants.alageaIntakeSubsystemConstants.initAngle);
    }
    setPower((pidController.calculate(getAngle())));
  }
}
