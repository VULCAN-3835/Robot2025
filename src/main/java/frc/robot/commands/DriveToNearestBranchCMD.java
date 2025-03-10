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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ChassisConstants;
import frc.robot.subsystems.ChassisSubsystem;

public class DriveToNearestBranchCMD extends Command {
  private final ChassisSubsystem chassis;
  private final boolean targetIsLeft;

  private Pose2d targetPose;
  private Trajectory trajectory;
  private double startTime;

  // Define maximum velocity and acceleration for the trajectory (in meters per second and m/s^2)
  private final double maxVelocity = 2.5;
  private final double maxAcceleration = 3;

  // Holonomic Drive Controller with PID for translation and rotation
  private final HolonomicDriveController controller = new HolonomicDriveController(
      new PIDController(2.0, 0, 0), // X-direction PID
      new PIDController(2.0, 0, 0), // Y-direction PID
      new ProfiledPIDController(1.5, 0, 0, new TrapezoidProfile.Constraints(maxVelocity, maxAcceleration)) // Theta PID
  );

  public DriveToNearestBranchCMD(ChassisSubsystem chassis, boolean left) {
    this.targetIsLeft = left;
    this.chassis = chassis;
    addRequirements(chassis);
  }

  @Override
  public void initialize() {
    // Get the current pose from the chassis subsystem
    Pose2d currentPose = chassis.getPose();
    this.targetPose = this.targetIsLeft ? chassis.getNearestLeft() : chassis.getNearestRight();
    // Optionally reset odometry if needed
    // chassis.resetOdometry(currentPose); // Uncomment if odometry drift is an issue

    // Configure trajectory settings
    TrajectoryConfig config = new TrajectoryConfig(maxVelocity, maxAcceleration)
        .setKinematics(ChassisConstants.kDriveKinematics); // Ensure kinematics are accounted for

    // Generate a trajectory from the current pose to the target pose
    System.out.println(currentPose);
    System.out.println(targetPose);
    trajectory = TrajectoryGenerator.generateTrajectory(List.of(currentPose, targetPose), config);

    // Start the timer
    startTime = Timer.getFPGATimestamp();
  }

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
