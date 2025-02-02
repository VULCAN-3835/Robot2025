// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Util.ElevatorStates;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.EndAccessorySubsystem;
import frc.robot.subsystems.EndAccessorySubsystem.DropAngles;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ScoreCoral extends SequentialCommandGroup {
  public ScoreCoral(EndAccessorySubsystem endAccessorySubsystem, ElevatorSubsystem elevatorSubsystem, DropAngles dropingLevel, ElevatorStates elevatorStates) {
    addCommands(

      new InstantCommand(() -> endAccessorySubsystem.setAngle(dropingLevel)),

      elevatorSubsystem.setLevelElevatorCommand(elevatorStates),

      new WaitUntilCommand(() -> endAccessorySubsystem.isAtSetpoint()),

      new WaitUntilCommand(() -> elevatorSubsystem.isAtSetHeight()),

      new CoralReleaseCommand(endAccessorySubsystem, dropingLevel),

      new InstantCommand(() -> endAccessorySubsystem.setAngle(DropAngles.restingAngle)),

      new InstantCommand(() -> elevatorSubsystem.setRest()),

      new WaitUntilCommand(() -> endAccessorySubsystem.isAtSetpoint()),

      new WaitUntilCommand(() -> elevatorSubsystem.isAtSetHeight())
    );
  }
}