// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static edu.wpi.first.units.Units.Centimeter;
import static edu.wpi.first.units.Units.Centimeters;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants.ElevatorConstant;
import frc.robot.Util.ElevatorStates;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.EndAccessorySubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ThrowAlgeaAutoCommand extends SequentialCommandGroup {
  /** Creates a new ThrowAlgeaAutoCommand. */
  public ThrowAlgeaAutoCommand(ElevatorSubsystem elevatorSubsystem,EndAccessorySubsystem endAccessorySubsystem, ElevatorStates elevatorStates, ElevatorConstant elevatorConstant){

    addCommands(
      new InstantCommand(() -> elevatorSubsystem.setLevel(elevatorStates)),
      new WaitUntilCommand(()->elevatorSubsystem.getDistance().minus(Centimeters.of(elevatorConstant.enumDistance(elevatorStates).in(Centimeter))).lt(Centimeters.of(elevatorConstant.errorTollerance.in(Centimeter)))),
      new InstantCommand(()-> endAccessorySubsystem.gripperIntake()),
      new InstantCommand(()-> endAccessorySubsystem.gripperRelease()),
      new InstantCommand(()-> elevatorSubsystem.setLevel(elevatorStates.rest))
    );
  
  }
}
