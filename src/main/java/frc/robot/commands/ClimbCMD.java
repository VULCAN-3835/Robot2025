// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.Constants.ClimbSubsystemConstants;;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ClimbCMD extends SequentialCommandGroup {
  /** Creates a new ClimbCMD. */
  public ClimbCMD(ClimbSubsystem climbSubsystem) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    
    addCommands(
      // 1. setting the motor to its desired speed so it will climb
      new InstantCommand(() -> climbSubsystem.setMotor(ClimbSubsystemConstants.climbMotorPower)),

      //2. waiting until we finish to climb
      new WaitUntilCommand(()-> climbSubsystem.getLimitswitch()),

      //3. stopping the motor when we finish to climb
      new InstantCommand(()-> climbSubsystem.setMotor(0)),

      //4. resets the position of the endcoder
      new InstantCommand(()-> climbSubsystem.resetPosition())
    );
  }
}
