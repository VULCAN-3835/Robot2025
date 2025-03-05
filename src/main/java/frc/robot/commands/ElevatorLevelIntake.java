// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Util.ElevatorStates;
import frc.robot.subsystems.ChassisSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.EndAccessorySubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ElevatorLevelIntake extends SequentialCommandGroup {
  /** Creates a new ElevatorLevelIntake. */
  public ElevatorLevelIntake(ElevatorSubsystem elevatorSubsystem,EndAccessorySubsystem endAccessorySubsystem,ChassisSubsystem chassisSubsystem) {
    
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());

    addCommands(

      // 1. sets the height of the elevator to the desired height 
      elevatorSubsystem.setLevelElevatorCommand(ElevatorStates.source),

      // 2. waits until the elevator is in the desired height
      new WaitUntilCommand(()->elevatorSubsystem.isAtSetpoint()),

      // 3. collects the coral 
      new CoralCollectCommand(endAccessorySubsystem),


      // 4. returns the elevator its resting state  
        new InstantCommand(()-> elevatorSubsystem.setLevel(ElevatorStates.rest))
      
    );
  }
}
