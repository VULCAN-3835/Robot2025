// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ClimbSubsystem extends SubsystemBase {
  /** Creates a new ClimbSubsystem. */
  TalonFX climbMotor;
  DigitalInput climbLimitSwitch;

  public ClimbSubsystem() {
    this.climbMotor = new TalonFX(Constants.ClimbSubsystemConstants.climbMotorPort);
    this.climbLimitSwitch = new DigitalInput(Constants.ClimbSubsystemConstants.limitSwitchPort);
  }

  // sets the power of climbMotor
  public void setMotor(double power) {
    climbMotor.set(power);
  }

  // returns the value of the limit switch
  public boolean getLimitswitch() {
    boolean valueOfLimitSwitch = climbLimitSwitch.get();
    return valueOfLimitSwitch;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (getLimitswitch()) {
      setMotor(Constants.ClimbSubsystemConstants.stoppingMotorSpeed);

    }

  }
}
