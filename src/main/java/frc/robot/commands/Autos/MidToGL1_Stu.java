// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autos;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Util.ElevatorStates;
import frc.robot.commands.ElevatorLevelScoreCMD;
import frc.robot.commands.ScoreL1;
import frc.robot.subsystems.ChassisSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.EndAccessorySubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class MidToGL1_Stu extends SequentialCommandGroup {
  /** Creates a new MidToGL4_Stu. */
  ChassisSubsystem chassis;
  public MidToGL1_Stu(ChassisSubsystem chassis,ElevatorSubsystem elevatorSubsystem,EndAccessorySubsystem endAccessorySubsystem) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    this.chassis = chassis;
    addRequirements(chassis);

    addCommands(
      new InstantCommand(()->chassis.drive(-1.5, 0, 0, false)),
      new WaitCommand(4),
      new InstantCommand(()->chassis.drive(0, 0, 0, false)),
      new ScoreL1(endAccessorySubsystem)
    );
  }
}
