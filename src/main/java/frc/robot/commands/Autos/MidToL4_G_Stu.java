// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autos;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Util.ElevatorStates;
import frc.robot.commands.ElevatorLevelScoreCMD;
import frc.robot.subsystems.ChassisSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.EndAccessorySubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class MidToL4_G_Stu extends SequentialCommandGroup {
  /** Creates a new MidToL4_G. */
  ChassisSubsystem chassis;
  public MidToL4_G_Stu(ChassisSubsystem chassis,ElevatorSubsystem elevatorSubsystem,EndAccessorySubsystem endAccessorySubsystem) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    this.chassis = chassis;
    addRequirements(chassis);
    addCommands(
      
      // 1. Drives 1 meter per second 
      new InstantCommand(()->chassis.drive(-1, 0, 0, false)),

      // 2. Waits 2 seconds until the robot is in frfont of the reef 
      new WaitCommand(2),

      // 3. Stops the robot
      new InstantCommand(()->chassis.drive(0, 0, 0, false)),

      // 4. Scores L4
      new ElevatorLevelScoreCMD(elevatorSubsystem, endAccessorySubsystem, ElevatorStates.coralL4)
    );
  }
}
