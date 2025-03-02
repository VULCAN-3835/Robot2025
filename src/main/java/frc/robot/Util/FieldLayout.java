// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Util;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Centimeters;

import java.util.ArrayList;
import java.util.Optional;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants.ChassisConstants.distanceConstants;

public final class FieldLayout {
    public static AprilTagFieldLayout aprilTagFieldLayout = AprilTagFieldLayout
            .loadField(AprilTagFields.k2025Reefscape);

    private static Pose2d getPoseAlignedToDrive(int tagID, Distance offset, Rotation2d rotationOffset) {
        Optional<Pose3d> optionalTagPose = aprilTagFieldLayout.getTagPose(tagID);
        if (!optionalTagPose.isPresent()) {
            DriverStation.reportError("AprilTag " + tagID + " not found.", false);
            return new Pose2d();
        }
        Pose2d tagPose = optionalTagPose.get().toPose2d();
        Rotation2d tagRotation = tagPose.getRotation();

        // Adjust offset to match the tag's orientation
        Translation2d offsetTranslation = new Translation2d(offset.in(Meters) + 0.5, tagRotation.plus(rotationOffset));
        Pose2d adjustedPose = new Pose2d(
                tagPose.getTranslation().minus(offsetTranslation),
                tagRotation.plus(rotationOffset));
        return adjustedPose;
    }

    public static Pose2d getDriveToReefPose(int tagID, Distance offset) {
        return getPoseAlignedToDrive(tagID, offset, Rotation2d.fromDegrees(180));
    }

    public static Pose2d getDriveToProcessorPose(Distance offset) {
        boolean isBlue = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue;
        int tagID = isBlue ? 16 : 3;
        return getPoseAlignedToDrive(tagID, offset, Rotation2d.fromDegrees(90));
    }

    public static Pose2d getNearestBranchPose(Pose2d currentPose, boolean right) {
        ArrayList<Pose2d> reefPoses = new ArrayList<>();

        reefPoses.add(getDriveToReefPose(18, distanceConstants.topReefDistance));
        reefPoses.add(getDriveToReefPose(22, distanceConstants.topRightReefDistance));
        reefPoses.add(getDriveToReefPose(20, distanceConstants.topLeftReefDistance));
        reefPoses.add(getDriveToReefPose(19, distanceConstants.bottomLeftReefDistance));
        reefPoses.add(getDriveToReefPose(17, distanceConstants.bottomRightReefDistance));
        reefPoses.add(getDriveToReefPose(7, distanceConstants.bottomReefDistance));

        return currentPose.nearest(reefPoses);
    }

    public static Pose2d getDesiredPoseBehindTag(double distanceMeters) {

        // Determine alliance and choose tag ID based on alliance color
        boolean isBlue = DriverStation.getAlliance().get() == DriverStation.Alliance.Blue;
        int tagId = isBlue ? 13 : 1;

        // Retrieve the tag's pose from the field layout (returns an Optional)
        Optional<Pose3d> tagPoseOpt = aprilTagFieldLayout.getTagPose(tagId);
        if (!tagPoseOpt.isPresent()) {
            System.err.println("AprilTag with ID " + tagId + " not found!");
            return new Pose2d(); // Default pose or handle error appropriately
        }

        // Convert the tag's Pose3d to a Pose2d
        Pose2d tagPose = tagPoseOpt.get().toPose2d();

        // Retrieve the tag's heading in degrees
        double thetaDegrees = tagPose.getRotation().getDegrees();

        // Convert the angle to radians for trigonometric calculations
        double thetaRadians = Math.toRadians(thetaDegrees);

        // Calculate the new position by moving 'distanceMeters' behind the tag
        // (opposite its heading)
        double desiredX = tagPose.getX() - distanceMeters * Math.cos(thetaRadians);
        double desiredY = tagPose.getY() - distanceMeters * Math.sin(thetaRadians);

        // Calculate the desired rotation: face the tag by adding 180 degrees
        double desiredRotationDegrees = thetaDegrees + 180;
        Rotation2d desiredRotation = Rotation2d.fromDegrees(desiredRotationDegrees);

        // Return the new pose behind the tag, with position in meters and rotation in
        // degrees
        return new Pose2d(desiredX, desiredY, desiredRotation);
    }

    public static Pose2d getNearestSource(Pose2d currentPose){
        boolean isBlue = DriverStation.getAlliance().get() == DriverStation.Alliance.Blue;
        ArrayList<Pose2d> sourcePoses = new ArrayList<>();
        if (isBlue) {
            sourcePoses.add(new Pose2d(1.262,6.853,aprilTagFieldLayout.getTagPose(13 ).get()
            .getRotation().toRotation2d().plus(Rotation2d.fromDegrees(180))));
            sourcePoses.add(new Pose2d(1.551,1.011,aprilTagFieldLayout.getTagPose(12 ).get()
            .getRotation().toRotation2d().plus(Rotation2d.fromDegrees(180))));
            return currentPose.nearest(sourcePoses);
        } else{
            sourcePoses.add(new Pose2d(15.917,0.764,aprilTagFieldLayout.getTagPose(1 ).get()
            .getRotation().toRotation2d().plus(Rotation2d.fromDegrees(180))));
            sourcePoses.add(new Pose2d(16.237,6.873,aprilTagFieldLayout.getTagPose(2 ).get()
            .getRotation().toRotation2d().plus(Rotation2d.fromDegrees(180))));
            return currentPose.nearest(sourcePoses);

        }
    }
    public static Pose2d getNearestBranchRight(Pose2d currentPose) {
        
        boolean isBlue = DriverStation.getAlliance().get() == DriverStation.Alliance.Blue;

        ArrayList<Pose2d> rightBranches = new ArrayList<>();

        if (isBlue) {
            // bottom side
            rightBranches.add(new Pose2d(3.047, 3.850, aprilTagFieldLayout.getTagPose(18 ).get()
                    .getRotation().toRotation2d().plus(Rotation2d.fromDegrees(180))));
            // bottom right
            rightBranches.add(new Pose2d(3.935, 2.838, aprilTagFieldLayout.getTagPose(17 ).get()
                    .getRotation().toRotation2d().plus(Rotation2d.fromDegrees(180))));
            // top right
            rightBranches.add(new Pose2d(5.349, 2.848, aprilTagFieldLayout.getTagPose(22 ).get()
                    .getRotation().toRotation2d().plus(Rotation2d.fromDegrees(180))));

            // top
            rightBranches.add(new Pose2d(6.092, 4.200, aprilTagFieldLayout.getTagPose(21 ).get()
                    .getRotation().toRotation2d().plus(Rotation2d.fromDegrees(180))));
            // top left
            rightBranches.add(new Pose2d(5.101, 5.377, aprilTagFieldLayout.getTagPose(20).get()
                    .getRotation().toRotation2d().plus(Rotation2d.fromDegrees(180))));
            // bottom left
            rightBranches.add(new Pose2d(3.594, 5.315, aprilTagFieldLayout.getTagPose( 19 ).get()
                    .getRotation().toRotation2d().plus(Rotation2d.fromDegrees(180))));
            return currentPose.nearest(rightBranches);

        } else {
            // bottom side
            rightBranches.add(new Pose2d(14.709, 4.200, aprilTagFieldLayout.getTagPose( 7).get()
                    .getRotation().toRotation2d().plus(Rotation2d.fromDegrees(180))));

            // bottom right
            rightBranches.add(new Pose2d(13.626, 5.418, aprilTagFieldLayout.getTagPose( 8).get()
                    .getRotation().toRotation2d().plus(Rotation2d.fromDegrees(180))));

            // top right
            rightBranches.add(new Pose2d(12.139, 5.294, aprilTagFieldLayout.getTagPose( 9).get()
                    .getRotation().toRotation2d().plus(Rotation2d.fromDegrees(180))));

            // top
            rightBranches.add(new Pose2d(11.603, 3.881, aprilTagFieldLayout.getTagPose(10).get()
                    .getRotation().toRotation2d().plus(Rotation2d.fromDegrees(180))));

            // top left
            rightBranches.add(new Pose2d(12.490, 2.642, aprilTagFieldLayout.getTagPose( 11).get()
                    .getRotation().toRotation2d().plus(Rotation2d.fromDegrees(180))));
            // bottom left
            rightBranches.add(new Pose2d(13.987, 2.756, aprilTagFieldLayout.getTagPose(6).get()
                    .getRotation().toRotation2d().plus(Rotation2d.fromDegrees(180))));
            return currentPose.nearest(rightBranches);
        }
    }
}
