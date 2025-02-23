// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Centimeter;


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
import edu.wpi.first.wpilibj.Encoder;
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
  private final TalonFX elevatorMotor;
  private final DigitalInput closeLimitSwitch;
  private final ElevatorFeedforward elevatorFeedforward;
  private final ProfiledPIDController profiledPIDController;
  private final Constraints constraints;

  //the sysID objects
  // private final SysIdRoutine sysIdRoutine;
  // private final Config config;



  private double currentPosition = 0;



  public ElevatorSubsystem() {
    this.elevatorMotor = new TalonFX(ElevatorConstant.elevatorMotorID);
    this.closeLimitSwitch = new DigitalInput(ElevatorConstant.limitSwitchID);


    this.elevatorFeedforward = new ElevatorFeedforward(ElevatorConstant.kS, ElevatorConstant.kG, ElevatorConstant.kV);
    this.constraints = new Constraints(ElevatorConstant.maxVelocity, ElevatorConstant.maxAcceleration);

    this.profiledPIDController = new ProfiledPIDController(ElevatorConstant.ProfiledkP, ElevatorConstant.ProfiledkI,
     ElevatorConstant.ProfiledkD, constraints);
    this.profiledPIDController.setTolerance(ElevatorConstant.pidTolerence);

  //   //SysID configs: the first param is how mach volts will it go up in how much time,
  //   //seconds one is how much for the quastatics tests and
  //   //the third one is how many seconds to finish for safety

  //   this.config = new Config( Volts.of(0.05).per(Millisecond), Volts.of(3), Seconds.of(6));
  //   this.sysIdRoutine = new SysIdRoutine(config,
  //    new SysIdRoutine.Mechanism(this::setVoltage,
  //     Log->{
  //       Log.motor("elevator Motor")
  //       .voltage(elevatorMotor.getMotorVoltage().getValue())
  //       .linearPosition(Meters.of(getDistance().in(Meters)))
  //       .linearVelocity(MetersPerSecond.of(elevatorMotor.getVelocity().getValue().in(RotationsPerSecond)*11.5));

  //   }, this));
  }

  // //SysID command for quasistatics tests
  // public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
  // return this.sysIdRoutine.quasistatic(direction);
  // }
  // //SysID command for dynamic tests
  // public Command sysIdDynamic(SysIdRoutine.Direction direction) {
  //   return this.sysIdRoutine.dynamic(direction);
  // }

  // public void setVoltage(Voltage volts){
  //   elevatorMotor.setVoltage(volts.in(Volts));
  // }

  public void setLevel(ElevatorStates state) {
    profiledPIDController.setGoal(ElevatorConstant.enumDistance(state).in(Centimeter));
  }
  public boolean isAtSetpoint(){
    return profiledPIDController.atGoal();
  }

  // current height
  public Measure<DistanceUnit> getDistance() {
    Angle angle = (this.elevatorMotor.getPosition().getValue());
    return ElevatorConstant.distancePerRotation.timesDivisor(angle).times(-1);
  }

  public void setRest() {
    profiledPIDController.setGoal(ElevatorConstant.restDistance.in(Centimeter));
  }

  public boolean getCloseLimitSwitch() {
    return closeLimitSwitch.get();
  }

  public InstantCommand setLevelElevatorCommand(ElevatorStates elevatorStates) {
      return new InstantCommand(() -> this.setLevel(elevatorStates));
  }

  
  // elevator SysID to use this add this in the configureXboxBinding method in the robotContainter
  // xboxControllerDrive.a().whileTrue(elevatorSubsystem.sysIdDynamic(SysIdRoutine.Direction.kForward));
  // xboxControllerDrive.b().whileTrue(elevatorSubsystem.sysIdDynamic(SysIdRoutine.Direction.kReverse));
  // xboxControllerDrive.y().whileTrue(elevatorSubsystem.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
  // xboxControllerDrive.x().whileTrue(elevatorSubsystem.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));

  @Override
  public void periodic() {

    //the current position and the velocity of the elevator
    currentPosition = getDistance().in(Centimeter);


    //calculates the controlles output to the motors
    double motorOutput = -(profiledPIDController.calculate(currentPosition) + elevatorFeedforward.calculate(profiledPIDController.getSetpoint().velocity));

    // uses the velcity that has been calculated by the trapzoid
    // the sum of the output power by all of the motor controllers

    if (getDistance().in(Centimeter)<1 && motorOutput > 0) {
      elevatorMotor.set(0);
    } else {
      elevatorMotor.set(motorOutput);

    }

    if (getCloseLimitSwitch()) {
      currentPosition = 0;
    }

    SmartDashboard.putNumber("ElevatorSubsystem/distance of elevator", currentPosition);
    SmartDashboard.putBoolean("ElevatorSubsystem/low limit switch pressed", getCloseLimitSwitch());
    SmartDashboard.putNumber("ElevatorSubsystem/ setPoint", profiledPIDController.getGoal().position);
    SmartDashboard.putBoolean("ElevatorSubsystem/ is at setPoint", isAtSetpoint());
    SmartDashboard.putNumber("ElevatorSubsystem/ output",motorOutput);

  }
}
