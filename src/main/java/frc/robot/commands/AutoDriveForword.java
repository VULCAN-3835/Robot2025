// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants.ChassisConstants;
import frc.robot.subsystems.ChassisSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoDriveForword extends SequentialCommandGroup {
  /** Creates a new AutoDriveForword. */
  public AutoDriveForword(ChassisSubsystem chassis) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new InstantCommand(()-> System.out.println("works0")),
    new InstantCommand(()->chassis.drive(1, 0, 0, false), chassis),
    new WaitUntilCommand(1),
    new InstantCommand(()->chassis.drive(0, 0, 0, false), chassis),
    new InstantCommand(()-> System.out.println("works"))
    );
  }
}
