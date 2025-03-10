// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.Degrees;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimbSubsystemConstants;

public class ClimbSubsystem extends SubsystemBase {
  /** Creates a new ClimbSubsystem. */
  TalonFX climbMotor1;
  TalonFX climbMotor2;
  public ClimbSubsystem() {
    this.climbMotor1 = new TalonFX(ClimbSubsystemConstants.climbMotorPort1);
    this.climbMotor2 = new TalonFX(ClimbSubsystemConstants.climbMotorPort2);

    climbMotor2.setControl(new Follower(ClimbSubsystemConstants.climbMotorPort1, false));
  }

  

  // returns the angle of the arm
  public Angle getPositionAngle() {
    StatusSignal<Angle> currentPosition = climbMotor1.getPosition();
    return currentPosition.getValue().div(ClimbSubsystemConstants.motorRatio); 
  }

  // sets the power of climbMotor
  public void setMotor(double power) {
    climbMotor1.set(power);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Climb Subsystem/ encoder value ", getPositionAngle().in(Degrees));
  }
}
