package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ChassisSubsystem;

public class AlignToAprilTagCommand extends Command {
    private final ChassisSubsystem chassis;
    // Proportional gain for rotation correction (in degrees per degree error)
    private final double kRot = 0.05;
    // Constant forward speed in meters per second
    private final double kForward = 0.5;
    // Optionally, you could add a proportional gain for lateral correction if needed
    private final double kLateral = 0.0;

    /**
     * Constructs a command that aligns the robot to an AprilTag using the chassis’ Limelight.
     *
     * @param chassis The chassis subsystem that contains the Limelight and drive method.
     */
    public AlignToAprilTagCommand(ChassisSubsystem chassis) {
        this.chassis = chassis;
        addRequirements(chassis);
    }

    @Override
    public void execute() {
        // Use the LimelightUtil already declared in the chassis
        // If the Limelight doesn't see a valid target, stop the robot.
        if (!chassis.getCam().hasValidTarget()) {
            chassis.drive(0.0, 0.0, 0.0, true);
            return;
        }

        // Get the horizontal offset (tx) from the limelight (in degrees)
        double tx = chassis.getCam().getX();
        // (Optional) Use the vertical offset (ty) for lateral adjustments if desired.
        double ty = chassis.getCam().getY();

        // Compute the rotational velocity to reduce the tx error
        double rotVelocity = -kRot * tx;
        // For now, we use a constant forward (x) velocity.
        double xVelocity = kForward;
        // Optionally, add lateral correction if needed; here we set it to zero or use kLateral * ty.
        double yVelocity = kLateral * ty;

        // Drive the robot with the computed velocities (xVelocity, yVelocity, rotVelocity)
        // Field-oriented is set to true.
        chassis.drive(xVelocity, yVelocity, rotVelocity, true);
    }

    @Override
    public boolean isFinished() {
        // This command runs continuously until interrupted.
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        // Stop the robot when the command ends.
        chassis.drive(0.0, 0.0, 0.0, true);
    }
}
