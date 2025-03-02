package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.EndAccessorySubsystem;
import frc.robot.subsystems.EndAccessorySubsystem.DropAngles;



public class CoralReleaseCommand extends SequentialCommandGroup {
    public CoralReleaseCommand(EndAccessorySubsystem endAccessorySubsystem,DropAngles dropAngle) {

        addCommands(

            // 2. Activate the gripper to release the piece
            new InstantCommand(() -> endAccessorySubsystem.setPower(endAccessorySubsystem.getSpeed(dropAngle))),

            // 3. Wait until the piece is no longer detected by the piece sensor
            // new WaitUntilCommand(()->!endAccessorySubsystem.hasPiece())
            new WaitCommand(3),

            // 4. Stop the gripper and sets the angle to return to the rest state
            new InstantCommand(() -> endAccessorySubsystem.gripperStop())
            
        );    
    }
}