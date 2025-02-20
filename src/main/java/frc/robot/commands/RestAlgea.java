// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.AlgeaSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class RestAlgea extends ParallelCommandGroup {
  /** Creates a new RestAlgea. */
  public RestAlgea(AlgeaSubsystem algeaSubsystem) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      // 1. sets the angle of the system to the resting angle
      new InstantCommand(()-> algeaSubsystem.setRestAngle()),

      // 2. stops the motor of the algea 
      new InstantCommand(()-> algeaSubsystem.setPower(0))
    );
  }
}
