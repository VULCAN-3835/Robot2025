// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.AlageaSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ShootingAlageaCmd extends SequentialCommandGroup {
  /** Creates a new ShootingAlageaCmd. */
  public ShootingAlageaCmd(AlageaSubsystem alageaSubsystem) {
    // Add your commands in the addCommands() call, e.g
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(

        // 1. sets the subsystem in the predefined shooting angle
        new InstantCommand(() -> alageaSubsystem.setShootingAngle()),

        // 2. waits until the subsystem is at the desired angle
        new WaitUntilCommand(() -> alageaSubsystem.isSystemAtShootingAngle()),

        // 3. shoots the alagea
        new InstantCommand(() -> alageaSubsystem.shootAlagea()),

        // 4. checkes if the alagea was shot
        new WaitUntilCommand(() -> !alageaSubsystem.hasBall()),
        
        // 5. stops the motor and sets the subsystem in the predefined resting angle
        new InstantCommand(() -> {
          alageaSubsystem.setPower(0);
          alageaSubsystem.setRestAngle();
        })

    );
  }
}
