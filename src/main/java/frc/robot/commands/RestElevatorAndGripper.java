// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Util.ElevatorStates;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.EndAccessorySubsystem;
import frc.robot.subsystems.EndAccessorySubsystem.DropAngles;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class RestElevatorAndGripper extends SequentialCommandGroup {
  /** Creates a new RestElevatorAndGripper. */
  public RestElevatorAndGripper(ElevatorSubsystem elevatorSubsystem,EndAccessorySubsystem endAccessorySubsystem) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    
    addCommands(
      // 1. stops the gripper
      new InstantCommand(()-> endAccessorySubsystem.gripperStop()),

      //2. returns the gripper angle and the elevator to their resting state
      new ParallelCommandGroup(
        elevatorSubsystem.setLevelElevatorCommand(ElevatorStates.rest),
        new InstantCommand(() -> endAccessorySubsystem.setAngle(DropAngles.restingAngle))
      )    

    );
  }
}
