package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.EndAccessorySubsystem;

public class CoralCollectCommand extends SequentialCommandGroup {

    public CoralCollectCommand(EndAccessorySubsystem endAccessorySubsystem) {

        addCommands(
            
            // 1. Activate the gripper to start the intake process
            new InstantCommand(() -> endAccessorySubsystem.gripperIntake()),

            // 2. Wait until the piece is detected by the piece sensor
            endAccessorySubsystem.waitForCoral(),
    
            // 3. Stop the gripper once the piece is successfully collected
            new InstantCommand(() -> endAccessorySubsystem.gripperStop())
        );
    }
}