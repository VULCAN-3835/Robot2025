// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Centimeter;
import static edu.wpi.first.units.Units.Centimeters;
import static edu.wpi.first.units.Units.Millisecond;
import static edu.wpi.first.units.Units.Milliseconds;
import static edu.wpi.first.units.Units.Minute;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.hardware.TalonFX;
import com.pathplanner.lib.path.ConstraintsZone;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.DistanceUnit;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;
import frc.robot.Constants.ElevatorConstant;
import frc.robot.Util.ElevatorStates;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;


public class ElevatorSubsystem extends SubsystemBase {
  /** Creates a new ElevatorSubsystem. */
  private final TalonFX elevatorMotorRight;
  private final TalonFX elevatorMotorLeft;
  private final DigitalInput closeLimitSwitch;
  private final PIDController pidController;
  private final TrapezoidProfile trapezoidProfile;
  private final ElevatorFeedforward elevatorFeedforward;
  private final ProfiledPIDController profiledPIDController;
  private final Constraints constraints;
  private final SysIdRoutine sysIdRoutine;
  private Config config;

  private final double maxVelocity = 2;
  private final double maxAcceleration = 4;

  private double currentPosition = 0;
  private double currentVelocity = 0;

  private State trapzoidSetPoint = new State();


  public ElevatorSubsystem() {
    this.elevatorMotorLeft = new TalonFX(ElevatorConstant.motorLeftID); 
    this.elevatorMotorRight = new TalonFX(ElevatorConstant.motorRightID);
    this.closeLimitSwitch = new DigitalInput(ElevatorConstant.limitSwitchID);

    this.elevatorFeedforward = new ElevatorFeedforward(ElevatorConstant.kS, ElevatorConstant.kG, ElevatorConstant.kV);
    this.pidController = new PIDController(ElevatorConstant.kP, ElevatorConstant.kI, ElevatorConstant.kD);
    this.constraints = new Constraints(maxVelocity, maxAcceleration);
    this.profiledPIDController = new ProfiledPIDController(ElevatorConstant.ProfiledkP, ElevatorConstant.ProfiledkI,
     ElevatorConstant.ProfiledkD, constraints);
    this.trapezoidProfile = new TrapezoidProfile(constraints);

    this.config = new Config( Volts.of(0.05).per(Millisecond), Volts.of(3), Seconds.of(3));
    
    this.sysIdRoutine = new SysIdRoutine(config,
     new SysIdRoutine.Mechanism(this::setVoltage,
      Log->{
        elevatorMotorLeft.getMotorVoltage();
        elevatorMotorLeft.get();
        getCloseLimitSwitch();

    }, this));
  }

  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
  return this.sysIdRoutine.quasistatic(direction);
  }
  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return this.sysIdRoutine.dynamic(direction);
  }

  public void setVoltage(Voltage volts){
    elevatorMotorLeft.setVoltage(volts.in(Volts));
    elevatorMotorRight.setVoltage(-volts.in(Volts));
  }

  public void setLevel(ElevatorStates state) {
    double setPoint = ElevatorConstant.enumDistance(state).in(Centimeter);

    pidController.setSetpoint(setPoint);
    profiledPIDController.setGoal(setPoint);
    trapzoidSetPoint = new State(setPoint, 0);
  }
  public void setPower(double power){
    elevatorMotorLeft.set(power);
    elevatorMotorRight.set(-power);

  }

  // current height
  public Measure<DistanceUnit> getDistance() {
    Angle angle1= (this.elevatorMotorLeft.getPosition().getValue());
    Angle angle2 = (this.elevatorMotorRight.getPosition().getValue());
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

  @Override
  public void periodic() {

    //the current position and the velocity of the elevator
    currentPosition = getDistance().in(Centimeter);
    currentVelocity  = elevatorMotorLeft.getVelocity().getValue().in(RotationsPerSecond)*60;


    //calculates the controlles output to the motors
    double profiledPIDOutput = profiledPIDController.calculate(currentPosition);
    double pidOutput = pidController.calculate(currentPosition);
    State trapzoidOutput = trapezoidProfile.calculate(trapezoidProfile.totalTime(),new State(currentPosition, currentVelocity), trapzoidSetPoint);

    //uses the velcity that has been calculated by the trapzoid
    double feedForwardOutput = elevatorFeedforward.calculate(trapzoidOutput.velocity);

    // the sum of the output power by all of the motor controllers
    double power = profiledPIDOutput + pidOutput + feedForwardOutput + trapzoidOutput.velocity;

    if (getCloseLimitSwitch() && power < 0) {
      elevatorMotorLeft.set(0);
      elevatorMotorRight.set(0);
    } else {
      elevatorMotorLeft.set(power);
      elevatorMotorRight.set(-power);

    }
    if (getCloseLimitSwitch()) {
      currentPosition = 0;
    }
    SmartDashboard.putNumber("distance of elevator", currentPosition);
    SmartDashboard.putBoolean("low limit switch pressed", getCloseLimitSwitch());

  }
}
