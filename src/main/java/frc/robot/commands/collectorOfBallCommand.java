// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.EndAccessorySubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class collectorOfBallCommand extends SequentialCommandGroup{
  /** Creates a new collectorOfBallCommand. */
  public collectorOfBallCommand(EndAccessorySubsystem endAccessorySubsystem) {
            addCommands(
            
            // 1. Activate the gripper to start the intake process
            new InstantCommand(() -> endAccessorySubsystem.gripperIntake()),

            // 2. Wait until the piece is detected by the piece sensor
            endAccessorySubsystem.waitForCoral(),
    
            // 3. Stop the gripper once the piece is successfully collected
            new InstantCommand(() -> endAccessorySubsystem.gripperRest())
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