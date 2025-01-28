// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.Constants.ClimbSubsystemConstants;
// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class CloseClimbCMD extends SequentialCommandGroup {
  /** Creates a new CloseClimbCMD. */
  public CloseClimbCMD(ClimbSubsystem climbSubsystem) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      //1.sets the motor to the closing speed
      new InstantCommand( ()->climbSubsystem.setMotor(ClimbSubsystemConstants.closeClimbMotorSpeed)),
      //2.waiting until the motor closes
      new WaitUntilCommand(()->climbSubsystem.getPosition().getValue().gt(ClimbSubsystemConstants.degreesForDropping)),
      //3. stops the motor after it gets to its desired spot
      new InstantCommand(()->climbSubsystem.setMotor(0))
    );
  }
}
