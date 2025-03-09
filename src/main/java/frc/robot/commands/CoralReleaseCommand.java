package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
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
            new InstantCommand(()-> System.out.println("stop!!!!!!!!!!!!!!")),

            new WaitCommand( 0.5),

            // 3. Stop the gripper
            new InstantCommand(() -> endAccessorySubsystem.gripperStop())
        );
    }
}
