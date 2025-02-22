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
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ChassisSubsystem;
import edu.wpi.first.math.trajectory.Trajectory;

public class DriveToPoseCommand extends Command {
  private final ChassisSubsystem chassis;
  private final Pose2d targetPose;
  private Trajectory trajectory;
  private double startTime;

  // Define maximum velocity and acceleration for the trajectory (in meters per second and m/s^2)
  private final double maxVelocity = 0.5;
  private final double maxAcceleration = 0.5;

  // Create a HolonomicDriveController with PID controllers for translation and rotation
  private final HolonomicDriveController controller = new HolonomicDriveController(
      new PIDController(1.0, 0, 0),    // X-direction PID
      new PIDController(1.0, 0, 0),    // Y-direction PID
      new ProfiledPIDController(1.0, 0, 0,
          new TrapezoidProfile.Constraints(maxVelocity, maxAcceleration)) // Theta PID
  );

  public DriveToPoseCommand(ChassisSubsystem chassis, Pose2d targetPose) {
    this.chassis = chassis;
    this.targetPose = targetPose;
    addRequirements(chassis);
  }

  @Override
  public void initialize() {
    // Get the current pose from the chassis subsystem
    Pose2d currentPose = chassis.getPose();

    // Optionally reset odometry here if needed
    chassis.resetOdometry(currentPose);

    // Configure trajectory settings
    TrajectoryConfig config = new TrajectoryConfig(maxVelocity, maxAcceleration);
    
    // Generate a simple trajectory from current pose to target pose.
    // Here, we use a two-point trajectory. For more complex paths, add intermediate waypoints.
    trajectory = TrajectoryGenerator.generateTrajectory(
        List.of(currentPose, targetPose),
        config
    );

    // Reset the timer
    startTime = Timer.getFPGATimestamp();
  }

  @Override
  public void execute() {
    double elapsedTime = Timer.getFPGATimestamp() - startTime;
    // Ensure we don't sample beyond the trajectory duration.
    double totalTime = trajectory.getTotalTimeSeconds();
    // Sample the trajectory state and extract the desired pose.
    var state = trajectory.sample(Math.min(elapsedTime, totalTime));
    Trajectory.State desiredState = trajectory.sample(Math.min(elapsedTime, totalTime));

    // Pass the state directly to the calculate() method.
    ChassisSpeeds chassisSpeeds = controller.calculate(chassis.getPose(), desiredState, targetPose.getRotation());



    // Drive the robot. The boolean flag indicates whether speeds are field-relative.
    chassis.drive(chassisSpeeds, false);
  }

  @Override
  public boolean isFinished() {
    // End the command when the trajectory has been fully followed.
    return (Timer.getFPGATimestamp() - startTime) >= trajectory.getTotalTimeSeconds();
  }

  @Override
  public void end(boolean interrupted) {
    // Stop the robot when the command ends or is interrupted.
    chassis.drive(0, 0, 0, false);
  }
}
