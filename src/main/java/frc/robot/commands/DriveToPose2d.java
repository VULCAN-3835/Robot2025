// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.List;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ChassisConstants;
import frc.robot.subsystems.ChassisSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class DriveToPose2d extends Command {
  /** Creates a new DriveToPose2d. */
  private ChassisSubsystem chassis;

  private Pose2d targetPose;
  private Trajectory trajectory;
  private double startTime;

  private final double maxVelocity = 2.5;
  private final double maxAcceleration = 3;

  private final HolonomicDriveController controller = new HolonomicDriveController(
      new PIDController(2.0, 0, 0), // X-direction PID
      new PIDController(2.0, 0, 0), // Y-direction PID
      new ProfiledPIDController(1.5, 0, 0, new TrapezoidProfile.Constraints(maxVelocity, maxAcceleration)) // Theta PID
  );
  public DriveToPose2d(ChassisSubsystem chassis, Pose2d targetPose) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.chassis = chassis;
    this.targetPose = targetPose;

    addRequirements(chassis);
    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Pose2d currentPose = chassis.getPose();

    TrajectoryConfig config = new TrajectoryConfig(maxVelocity, maxAcceleration)
        .setKinematics(ChassisConstants.kDriveKinematics); 

    System.out.println(currentPose);
    System.out.println(targetPose);
    trajectory = TrajectoryGenerator.generateTrajectory(List.of(currentPose, targetPose), config);

    startTime = Timer.getFPGATimestamp();

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    double elapsedTime = Timer.getFPGATimestamp() - startTime;
    double totalTime = trajectory.getTotalTimeSeconds();

    // Sample the trajectory to get the desired state
    Trajectory.State desiredState = trajectory.sample(Math.min(elapsedTime, totalTime));

    // Compute chassis speeds using the controller
    ChassisSpeeds chassisSpeeds = controller.calculate(chassis.getPose(), desiredState, targetPose.getRotation());

    // Command the chassis to move
    chassis.drive(chassisSpeeds.unaryMinus(), false);
  }

  @Override
  public boolean isFinished() {
    // End command when the trajectory is complete
    return (Timer.getFPGATimestamp() - startTime) >= trajectory.getTotalTimeSeconds();
  }

  @Override
  public void end(boolean interrupted) {
    // Stop the robot when finished or interrupted
    chassis.drive(new ChassisSpeeds(0, 0, 0),false);
  }
}
