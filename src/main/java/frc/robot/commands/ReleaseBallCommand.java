// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.EndAccessorySubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ReleaseBallCommand extends SequentialCommandGroup{
  /** Creates a new ReleaseBallCommand. */
  public ReleaseBallCommand(EndAccessorySubsystem endAccessorySubsystem) {

        addCommands(
            // 1. Activate the gripper to release the piece
            new InstantCommand(() -> endAccessorySubsystem.gripperRelease(), endAccessorySubsystem),

            // 2. Wait until the piece is no longer detected by the piece sensor
            new WaitUntilCommand(() -> !endAccessorySubsystem.hasPiece()),

            // 3. Stop the gripper once the piece is successfully released
            new InstantCommand(() -> endAccessorySubsystem.gripperRest(), endAccessorySubsystem)
        );
  }
}




  /* Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}*/
