// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autos;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Util.ElevatorStates;
import frc.robot.subsystems.ChassisSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.EndAccessorySubsystem;
import frc.robot.commands.DriveToNearestBranchCMD;
import frc.robot.commands.ElevatorLevelScoreCMD;
import frc.robot.commands.RestElevatorAndGripper;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class MidTo_G_L4 extends SequentialCommandGroup {
  // Creates a new MidTo_G_L3. 

  public MidTo_G_L4(ChassisSubsystem chassisSubsystem, ElevatorSubsystem elevatorSubsystem, EndAccessorySubsystem endAccessorySubsystem) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new DriveToNearestBranchCMD(chassisSubsystem, false),
      new DriveToNearestBranchCMD(chassisSubsystem, false),
      new DriveToNearestBranchCMD(chassisSubsystem, false),
      new ElevatorLevelScoreCMD(elevatorSubsystem, endAccessorySubsystem, ElevatorStates.coralL4),// TODO: add race with second coral
      new RestElevatorAndGripper(elevatorSubsystem, endAccessorySubsystem)
    );
  }
}
