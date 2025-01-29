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
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants.ElevatorConstant;
import frc.robot.Util.ElevatorStates;

public class ElevatorSubsystem extends SubsystemBase {
  /** Creates a new ElevatorSubsystem. */
  private final TalonFX ElevatorMotorRight;
  private final TalonFX ElevatorMotorLeft;
  private final DigitalInput closeLimitSwitch;
  private final PIDController pidController;
  private Distance disSetLevel;

  public ElevatorSubsystem() {
<<<<<<< Updated upstream
    this.ElevatorMotorLeft = new TalonFX(ElevatorConstant.motorLeftID); 
    this.ElevatorMotorRight = new TalonFX(ElevatorConstant.motorRightID);
    this.closeLimitSwitch = new DigitalInput(ElevatorConstant.limitSwitchID);
    this.pidController = new PIDController(ElevatorConstant.kP, ElevatorConstant.kI, ElevatorConstant.kD);
=======
    this.ElevatorMotorLeft = new TalonFX(ElevatorConstant.motorLeftID);
    this.ElevatorMotorRight = new TalonFX(ElevatorConstant.motorRightID);
    this.closeLimitSwitch = new DigitalInput(ElevatorConstant.limitSwitchID);
    this.pidController = new PIDController(ElevatorConstant.kP, ElevatorConstant.kI, ElevatorConstant.kD);

>>>>>>> Stashed changes
  }

  public void setLevel(ElevatorStates state) {
    disSetLevel = (ElevatorConstant.enumDistance(state));
    pidController.setSetpoint(disSetLevel.in(Centimeter));

  }

  // current height.
  public Measure<DistanceUnit> getDistance() {
    Angle angle1 = (this.ElevatorMotorLeft.getPosition().getValue());
    Angle angle2 = (this.ElevatorMotorRight.getPosition().getValue());
    Angle avg = angle1.minus(angle2).div(2);
    return ElevatorConstant.distancePerRotation.timesDivisor(avg);
  }

  public void setRest() {
    pidController.setSetpoint(ElevatorConstant.restDistance.in(Centimeter));
  }

  public boolean getCloseLimitSwitch() {
    return closeLimitSwitch.get();
  }

  public InstantCommand setLevelElevatorCommand(ElevatorStates elevatorStates) {
    return new InstantCommand(() -> this.setLevel(elevatorStates));
  }

  public Command waitForLevel() {
    return new WaitUntilCommand(
        () -> this.getDistance().minus(disSetLevel).abs(Centimeter) < ElevatorConstant.errorTollerance.in(Centimeter));
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
