package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Util.ElevatorStates;
import frc.robot.subsystems.EndAccessorySubsystem;

public class CoralReleaseCommand extends SequentialCommandGroup {
    public CoralReleaseCommand(EndAccessorySubsystem endAccessorySubsystem, ElevatorStates elevatorStates) {
        InstantCommand releaseCommand;

        if (elevatorStates == ElevatorStates.coralL1) {
            releaseCommand = new InstantCommand(() -> endAccessorySubsystem.setPower(0.1));
        } else {
            releaseCommand = new InstantCommand(() -> endAccessorySubsystem.gripperOut());
        }

        addCommands(
            // 1. Activate the gripper to release the piece
            releaseCommand,

            // 2. Wait until the piece is no longer detected by the piece sensor
            new WaitUntilCommand(() -> !endAccessorySubsystem.hasPiece()),

            new WaitUntilCommand(elevatorStates == ElevatorStates.coralL1 ? 0.5 : 0),

            // 3. Stop the gripper
            new InstantCommand(() -> endAccessorySubsystem.gripperStop())
        );
    }
}
