// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.AlgeaSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ShootingAlgeaCmd extends SequentialCommandGroup {
  /** Creates a new ShootingAlgeaCmd. */
  public ShootingAlgeaCmd(AlgeaSubsystem algeaSubsystem) {
    // Add your commands in the addCommands() call, e.g
    // addCommands(new FooCommand(), new BarCommand());
    addRequirements(algeaSubsystem);
    addCommands(

        // 1. sets the subsystem in the predefined shooting angle
        new InstantCommand(() -> algeaSubsystem.setShootingAngle()),

        // 2. waits until the subsystem is at the desired angle
        new WaitUntilCommand(() -> algeaSubsystem.isSystemAtShootingAngle()),

        // 3. shoots the algea
        new InstantCommand(() -> algeaSubsystem.shootAlgea()),

        // 4. checkes if the algea was shot
        new WaitUntilCommand(() -> !algeaSubsystem.hasBall()),
        
        // 5. stops the motor and sets the subsystem in the predefined resting angle
        new InstantCommand(() -> {
          algeaSubsystem.setPower(0);
          algeaSubsystem.setRestAngle();
        })

    );
  }
}
