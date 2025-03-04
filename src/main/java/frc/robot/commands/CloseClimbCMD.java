// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants.ClimbSubsystemConstants;
import frc.robot.subsystems.ClimbSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class CloseClimbCMD extends SequentialCommandGroup {
  /** Creates a new CloseClimbCMD. */

  public CloseClimbCMD(ClimbSubsystem climbSubsystem) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addRequirements(climbSubsystem);
    addCommands(
      //1. set the motor to the close motor speed
      new InstantCommand(()-> climbSubsystem.setMotor(ClimbSubsystemConstants.closeClimbMotorPower)),
      //2. waits until the degrees are the degrees of the closed arm
      new WaitUntilCommand(() -> climbSubsystem.getPositionAngle().gt(ClimbSubsystemConstants.degreesForOpen)),
      //3. stops the motor
      new InstantCommand(()-> climbSubsystem.setMotor(0))
      );
  }
}
