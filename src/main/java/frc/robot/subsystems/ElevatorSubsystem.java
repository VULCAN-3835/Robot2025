// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Centimeter;
import static edu.wpi.first.units.Units.Meter;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.DistanceUnit;
import edu.wpi.first.units.LinearVelocityUnit;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.VelocityUnit;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
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
  private Distance disSetLevel;
  private ProfiledPIDController profilePIDController;
  private ElevatorFeedforward elevatorFeedforward;
  

  public ElevatorSubsystem() {
    this.ElevatorMotorLeft = new TalonFX(ElevatorConstant.motorLeftID); 
    this.ElevatorMotorRight = new TalonFX(ElevatorConstant.motorRightID);
    this.closeLimitSwitch = new DigitalInput(ElevatorConstant.limitSwitchID);
    this.profilePIDController = new ProfiledPIDController(ElevatorConstant.kP, ElevatorConstant.kI, ElevatorConstant.kD, new TrapezoidProfile.Constraints(5, 10));
      profilePIDController.setGoal(0);
      new TrapezoidProfile.Constraints(ElevatorConstant.kMaxVelocity, ElevatorConstant.kMaxAcceleration);
      this.elevatorFeedforward =  new ElevatorFeedforward(ElevatorConstant.kS, ElevatorConstant.kG, ElevatorConstant.kV);
  }

  public void setLevel(ElevatorStates state) {
    disSetLevel = (ElevatorConstant.enumDistance(state));
    profilePIDController.setGoal(ElevatorConstant.enumDistance(state).in(Centimeter));

  }
  public void setPower(double power){
    ElevatorMotorLeft.set(power);
    ElevatorMotorRight.set(-power);

   // ElevatorMotorLeft.setVoltage(
      //m_controller.calculate(m_encoder.getDistance())
      //+ m_feedforward.calculate(m_controller.getSetpoint().velocity));

  }

  // current height.
  public Measure<DistanceUnit> getDistance() {
    Angle angle1 = (this.ElevatorMotorLeft.getPosition().getValue());
    Angle angle2 = (this.ElevatorMotorRight.getPosition().getValue());
    Angle avg = angle1.minus(angle2).div(2);
    return ElevatorConstant.distancePerRotation.timesDivisor(avg);
  }

  public void setRest() {
    this.setLevel(ElevatorStates.rest);
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
     double power = profilePIDController.calculate(getDistance().in(Centimeter))+ elevatorFeedforward.calculate(profilePIDController.getSetpoint().velocity);
     if (getCloseLimitSwitch() && power < 0) {
      setPower(0);
     } else {
      setPower(power);

     }
  }
}
