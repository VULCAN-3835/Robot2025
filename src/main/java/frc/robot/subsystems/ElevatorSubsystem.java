// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Centimeter;



import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.DistanceUnit;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstant;
import frc.robot.Util.ElevatorStates;

public class ElevatorSubsystem extends SubsystemBase {
  /** Creates a new ElevatorSubsystem. */
  private final TalonFX ElevatorMotorRight;
  private final TalonFX ElevatorMotorLeft;
  private final DigitalInput closeLimitSwitch;
  private final PIDController pidController;

  public ElevatorSubsystem() {
    this.ElevatorMotorLeft = new TalonFX(ElevatorConstant.motorLeftPort); 
    this.ElevatorMotorRight = new TalonFX(ElevatorConstant.motorRightPort);
    this.closeLimitSwitch = new DigitalInput(ElevatorConstant.limitSwitchPort);
    this.pidController = new PIDController(ElevatorConstant.kP, ElevatorConstant.kI, ElevatorConstant.kD);
  }

  public void setLevel(ElevatorStates state) {
    pidController.setSetpoint(ElevatorConstant.enumDistance(state).in(Centimeter));

  }

  // current height.
  public Measure<DistanceUnit> getDistance() {
    Angle angle1= (this.ElevatorMotorLeft.getPosition().getValue());
    Angle angle2 = (this.ElevatorMotorRight.getPosition().getValue());
    Angle avg = angle1.minus(angle2).div(2);
    return ElevatorConstant.distancePerRotation.timesDivisor(avg);
  }

  public void setRest() {
    pidController.setSetpoint(0);
  }

  public boolean getCloseLimitSwitch() {
    return closeLimitSwitch.get();
  }
  public InstantCommand setLevelElevatorCommand(ElevatorStates elevatorStates) {
      return new InstantCommand(() -> this.setLevel(elevatorStates));
  }

  @Override
  public void periodic() {
    double power = pidController.calculate(getDistance().in(Centimeter));
    if (getCloseLimitSwitch() && power < 0) {
      ElevatorMotorLeft.set(0);
      ElevatorMotorRight.set(0);
    } else {
      ElevatorMotorLeft.set(power);
      ElevatorMotorRight.set(-power);

    }
  }
}
