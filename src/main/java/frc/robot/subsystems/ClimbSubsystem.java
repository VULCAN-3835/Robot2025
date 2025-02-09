// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ClimbSubsystemConstants;

public class ClimbSubsystem extends SubsystemBase {
  /** Creates a new ClimbSubsystem. */
  TalonFX climbMotor;
  DigitalInput lowClimbLimitSwitch; 

  public ClimbSubsystem() {
    this.climbMotor = new TalonFX(Constants.ClimbSubsystemConstants.climbMotorPort);
    this.lowClimbLimitSwitch = new DigitalInput(Constants.ClimbSubsystemConstants.limitSwitchPort);
  }

  // sets the angle of the motor to 0
  public void resetPosition() {
    climbMotor.setPosition(0);
  }

  // returns the angle of the arm
  public Angle getPositionAngle() {
    StatusSignal<Angle> currentPosition = climbMotor.getPosition();
    return currentPosition.getValue().div(ClimbSubsystemConstants.motorRatio); 
  }

  // sets the power of climbMotor
  public void setMotor(double power) {
    climbMotor.set(power);
  }

  // returns the value of the limit switch
  public boolean getLimitswitch() {
    return lowClimbLimitSwitch.get();
  }

  @Override
  public void periodic() {
    // if (getLimitswitch() && climbMotor.get() < 0) {
    //   setMotor(0);
    // }
  }
}
