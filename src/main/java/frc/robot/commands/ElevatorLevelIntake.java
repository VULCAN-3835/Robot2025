// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Util.ElevatorStates;
import frc.robot.subsystems.ChassisSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.EndAccessorySubsystem;
import frc.robot.subsystems.EndAccessorySubsystem.DropAngles;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ElevatorLevelIntake extends SequentialCommandGroup {
  /** Creates a new ElevatorLevelIntake. */
  public ElevatorLevelIntake(ChassisSubsystem chassisSubsystem,ElevatorSubsystem elevatorSubsystem,EndAccessorySubsystem endAccessorySubsystem) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addRequirements(chassisSubsystem,elevatorSubsystem,endAccessorySubsystem);
    addCommands(
      // 1. positioning itself in front of the april tag
      // new PositioningAprilTag(chassisSubsystem, false, true),

      // 2. sets the angle of the end accessory and the height of the elevator in the same time 
      new ParallelCommandGroup(
        elevatorSubsystem.setLevelElevatorCommand(ElevatorStates.source),
        new InstantCommand(()-> endAccessorySubsystem.setAngle(DropAngles.intakeAngle))),

      // 3. releases the coral 
      new CoralCollectCommand(endAccessorySubsystem),

      // 4. returns the elevator and the end accessory to their resting state  
      new ParallelCommandGroup(
        new InstantCommand(()-> elevatorSubsystem.setLevel(ElevatorStates.rest)),
        new InstantCommand(()-> endAccessorySubsystem.setAngle(DropAngles.restingAngle))
      )
    );
  }
}
