package frc.robot.commands;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.Trajectory.State;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ChassisSubsystem;
import org.photonvision.targeting.PhotonTrackedTarget;

import java.util.List;
import java.util.Optional;

public class PositioningAprilTag extends Command {
    private final ChassisSubsystem chassisSubsystem;
    private final HolonomicDriveController controller;
    private Trajectory trajectory;
    private double startTime;
    private boolean isMovingRight;
    private boolean intake;

    public PositioningAprilTag(ChassisSubsystem chassisSubsystem, boolean isMovingRight, boolean intake) {
        this.chassisSubsystem = chassisSubsystem;
        this.isMovingRight = isMovingRight;
        this.intake = intake;
        addRequirements(chassisSubsystem);

        // TODO: change pid values to actual values
        double xKp = 5, xKi = 0, xKd = 0;
        double yKp = 5, yKi = 0, yKd = 0;
        double rotKp = 5, rotKi = 0, rotKd = 0;
        double maxVelocity = 2.0, maxAcceleration = 2.0;

        this.controller = new HolonomicDriveController(
                new PIDController(xKp, xKi, xKd),
                new PIDController(yKp, yKi, yKd),
                new ProfiledPIDController(rotKp, rotKi, rotKd,
                        new TrapezoidProfile.Constraints(maxVelocity, maxAcceleration)));
    }

    @Override
    public void initialize() {
        
        //the distance that the robot goes left or right to score on the coral
        double leftOrRightDistanceInMeters = 1;

        startTime = Timer.getFPGATimestamp();
        int ID = chassisSubsystem.getCamera().getID();

        Optional<Pose3d> tagPoseOpt = chassisSubsystem.getCamera().getTagPose3d(ID);

        if (chassisSubsystem.getCamera().getTagPose3d(ID).isPresent() && !intake) {

            Pose2d tagPose = tagPoseOpt.get().toPose2d();
            Pose2d startPose = chassisSubsystem.getPose();

            // Generate a trajectory to move 1 meter in front of the tag and 1 meter to the
            // left or right of the tag
            Pose2d targetPose = new Pose2d(
                    tagPose.getX() - tagPose.getRotation().getCos()
                            + (isMovingRight ? -leftOrRightDistanceInMeters : leftOrRightDistanceInMeters) * tagPose.getRotation().getSin(),
                    tagPose.getY() - tagPose.getRotation().getSin()
                            + (isMovingRight ? leftOrRightDistanceInMeters : -leftOrRightDistanceInMeters) * tagPose.getRotation().getCos(),
                    tagPose.getRotation());

            TrajectoryConfig config = new TrajectoryConfig(2.0, 2.0);
            trajectory = TrajectoryGenerator.generateTrajectory(
                    List.of(startPose, targetPose), config);
        }

        // Generate a trajectory to move 1 meter in front of the tag
        else if (intake && chassisSubsystem.getCamera().getTagPose3d(ID).isPresent()) {
            Pose2d tagPose = tagPoseOpt.get().toPose2d();
            Pose2d startPose = chassisSubsystem.getPose();

            Pose2d targetPose = new Pose2d(
                    tagPose.getX() - tagPose.getRotation().getCos(),
                    tagPose.getY() - tagPose.getRotation().getSin(),
                    tagPose.getRotation());

            TrajectoryConfig config = new TrajectoryConfig(2.0, 2.0);
            trajectory = TrajectoryGenerator.generateTrajectory(List.of(startPose, targetPose), config);

        } else {
            trajectory = new Trajectory(); // Empty trajectory if no tag is found
        }
    }

    @Override
    public void execute() {
        if (trajectory.getTotalTimeSeconds() == 0) {
            chassisSubsystem.drive(0, 0, 0, false);
            return;
        }

        double elapsedTime = Timer.getFPGATimestamp() - startTime;
        State goal = trajectory.sample(elapsedTime);

        Optional<Pose3d> tagPoseOpt = chassisSubsystem.getCamera().getTagPose3d(chassisSubsystem.getCamera().getID());

        if (tagPoseOpt.isPresent()) {
            Pose2d tagPose = tagPoseOpt.get().toPose2d();

            ChassisSpeeds speeds = controller.calculate(
                    chassisSubsystem.getPose(),
                    goal.poseMeters,
                    goal.velocityMetersPerSecond,
                    tagPose.getRotation());

            chassisSubsystem.drive(speeds, true);
        } else {
            chassisSubsystem.drive(0, 0, 0, false);
        }
    }

    @Override
    public boolean isFinished() {
        return Timer.getFPGATimestamp() - startTime >= trajectory.getTotalTimeSeconds();
    }

    @Override
    public void end(boolean interrupted) {
        chassisSubsystem.drive(0, 0, 0, false);
    }
}
