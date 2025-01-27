// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.ClimbSubsystem;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Dropping extends SequentialCommandGroup {
  /** Creates a new Dropping. */
  public Dropping(ClimbSubsystem climbSubsystem) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        // 1. starting the motor
        new InstantCommand(() -> climbSubsystem.setMotor(Constants.ClimbSubsystemConstants.workingMotorSpeed)),
        // 2. checking if the limitswitch is pressed
        new WaitUntilCommand(() -> climbSubsystem.getLimitswitch()),
        // 3. stopping the motor
        new InstantCommand(() -> climbSubsystem.setMotor(Constants.ClimbSubsystemConstants.stoppingMotorSpeed)),
        // 4. setting the Endoder's position to 0
        new InstantCommand(() -> climbSubsystem.setPosition()),
        // 5. starting to drop from the cage at certain degrees 
        new InstantCommand(() -> climbSubsystem.setMotor(Constants.ClimbSubsystemConstants.droppingMotorSpeed)),
        // 6. waiting until the motor is at the right degrees
        new WaitUntilCommand(() -> climbSubsystem.getPosition().getValue().gt(Constants.ClimbSubsystemConstants.degreesForDropping)),
        // 7. stopping the motor
        new InstantCommand(() -> climbSubsystem.setMotor(Constants.ClimbSubsystemConstants.stoppingMotorSpeed)));
  }
}
