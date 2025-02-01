// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Util.OVCameraUtil;
import frc.robot.subsystems.ChassisSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class CenterRobotCmd extends SequentialCommandGroup {
  /** Creates a new CenterRobotCmd. */
  public CenterRobotCmd(OVCameraUtil ovCameraUtil, ChassisSubsystem chassisSubsystem) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new InstantCommand(() -> {
        double xError = ovCameraUtil.getAprilTagX(); 
        double turnSpeed = 0.02 * xError; // sets the speed based on the distance error
        turnSpeed = Math.max(-0.5, Math.min(0.5, turnSpeed)); // limits the turning speed
        chassisSubsystem.drive(0, 0, turnSpeed, true); // sets the turning speed in the drive
      }),
      new WaitUntilCommand(() -> Math.abs(ovCameraUtil.getAprilTagX()) < 1.5), // waits until the robot is centered enough
      new InstantCommand(() -> chassisSubsystem.drive(0, 0, 0, true))// stops the robot in place
    );
  }
}
