package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.EndAccessorySubsystem;

public class ReleaseCoralCommand extends SequentialCommandGroup {

    public ReleaseCoralCommand(EndAccessorySubsystem endAccessorySubsystem) {

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