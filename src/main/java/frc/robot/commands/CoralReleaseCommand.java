package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.EndAccessorySubsystem;



public class CoralReleaseCommand extends SequentialCommandGroup {
    public CoralReleaseCommand(EndAccessorySubsystem endAccessorySubsystem) {

        addCommands(

            // 1. Activate the gripper to release the piece
            new InstantCommand(() -> endAccessorySubsystem.gripperOut()),

            // 2. Wait until the piece is no longer detected by the piece sensor
            new WaitUntilCommand(()-> !endAccessorySubsystem.hasPiece()),

            // 3. Stop the gripper 
            new InstantCommand(() -> endAccessorySubsystem.gripperStop())
            
        );    
    }
}