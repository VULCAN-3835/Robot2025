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
public class MidToL3_Stu extends SequentialCommandGroup {
  /** Creates a new MidToL3_Stu. */
  ChassisSubsystem chassisSubsystem;
  public MidToL3_Stu(ChassisSubsystem chassisSubsystem,ElevatorSubsystem elevatorSubsystem,EndAccessorySubsystem endAccessorySubsystem) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    this.chassisSubsystem = chassisSubsystem;
    addRequirements(chassisSubsystem);
    addCommands(
      
      // 1. Drives the robot in 1 meters per second 
      new InstantCommand(()->chassisSubsystem.drive(-1, 0, 0, false)),

      // 2. Waits 2 secnods until the robot is infront of the reef
      new WaitCommand(2),

      // 3. Stops the robot 
      new InstantCommand(()->chassisSubsystem.drive(0, 0, 0, false)),

      // 4. Scores L3
      new ElevatorLevelScoreCMD(elevatorSubsystem, endAccessorySubsystem, ElevatorStates.coralL3)

    );
  }
}
