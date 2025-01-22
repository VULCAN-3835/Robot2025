// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;

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
    // checks if the system detects the ball

    return ballDetector.getVoltage() > Constants.alageaIntakeSubsystemConstants.ballDetectorThreshold;

  }

  public Angle getAngle() {
    // gets the current angle
    return getPosition.getValue();

  }

  private void setAngle(double targetAngle) {
    // sets the desired robot angle
    if (targetAngle >= Constants.alageaIntakeSubsystemConstants.minAngle && targetAngle <= Constants.alageaIntakeSubsystemConstants.maxAngle) {
      pidController.setSetpoint(targetAngle); 
    }
  }

  public void setRestAngle() {
    // sets the robot in the predefined resting angle
    setAngle(Constants.alageaIntakeSubsystemConstants.restAngle);

  }

  public void setCollectAngle() {
    //  sets the robot in the predefined collecting angle
    setAngle(Constants.alageaIntakeSubsystemConstants.collectingAngle);

  }

  public void setHoldAngle() {
    //  sets the robot in the predefined holding angle
    setAngle(Constants.alageaIntakeSubsystemConstants.holdAngle);

  }

  public void setPower(double power) {
    // checkes if the system is trying to go past the limitSwitch and if so, stops the motor
    if (limitSwitch.get() && pidController.calculate(getAngle().in(Degrees)) < 0){
      powerMotor.set(0);
   } else {
    powerMotor.set(power);
   }
  }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // checks if limitSwitch is true and if so, sets the angle to the intalizing position
    // also constantly checkes and updates the desired settings for the motor
    if(limitSwitch.get()){
      angleMotor.set(Constants.alageaIntakeSubsystemConstants.initAngle);
    }
    setPower((pidController.calculate(getAngle().in(Degrees))));
  }
}

