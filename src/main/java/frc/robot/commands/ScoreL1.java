// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.EndAccessorySubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ScoreL1 extends SequentialCommandGroup {
  /** Creates a new ScoreL1. */
  public ScoreL1(EndAccessorySubsystem endAccessorySubsystem) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      // 1. Sets the power of the end accessory to 45%
      new InstantCommand(()-> endAccessorySubsystem.setPower(0.45)),

      // 2. waits 1.5 until the game piece isn't in the system anymore
      new WaitCommand(1.5),

      // 3. Stops the grippper 
      new InstantCommand(()-> endAccessorySubsystem.gripperStop())
    );
  }
}
