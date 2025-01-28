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
public class ResetClimbing extends SequentialCommandGroup {
  /** Creates a new Dropping. */
  public ResetClimbing(ClimbSubsystem climbSubsystem) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        // 1. starting the motor
        new ClimbCMD(climbSubsystem),

        // 2. starting to drop from the cage at certain degrees 
        new InstantCommand(() -> climbSubsystem.setMotor(-Constants.ClimbSubsystemConstants.closeClimbMotorPower)),

        // 3. waiting until the motor is at the right degrees
        new WaitUntilCommand(() -> climbSubsystem.getPositionAngle().gt(Constants.ClimbSubsystemConstants.degreesForOpen)),

        // 4. stopping the motor
        new InstantCommand(() -> climbSubsystem.setMotor(0)));
  }
}
