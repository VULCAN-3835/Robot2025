// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;
import frc.robot.subsystems.AlageaSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class CollectingAlageaCmd extends SequentialCommandGroup {
  /** Creates a new CollectingAlageaCmd. */
  public CollectingAlageaCmd(AlageaSubsystem alageaIntakeSubsystem) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
    // 1. starts to collect the piece
    new InstantCommand(()-> alageaIntakeSubsystem.setPower(Constants.alageaSubsystemConstants.collectingPower)),
    // 2. waits until the sensor feels the piece 
    new WaitUntilCommand(()-> alageaIntakeSubsystem.hasBall()),
    // 3. stops the motor
    new InstantCommand(()-> alageaIntakeSubsystem.setPower(0))
    );
  }
}
