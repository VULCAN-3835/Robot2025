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
import frc.robot.Constants.ChassisConstants.FieldLayoutConstants;

public final class FieldLayout {
        public static AprilTagFieldLayout aprilTagFieldLayout = AprilTagFieldLayout
            .loadField(AprilTagFields.k2025Reefscape);

        public static boolean isBlue = DriverStation.getAlliance().get() == DriverStation.Alliance.Blue;

    

    public static Pose2d getNearestBranchRight(Pose2d currentPose) {
        ArrayList<Pose2d> rightBranches = new ArrayList<>();

        // bottom side
        rightBranches.add(new Pose2d(FieldLayoutConstants.aBranchX, FieldLayoutConstants.aBranchY,
                aprilTagFieldLayout.getTagPose(isBlue?18:7).get()
                        .getRotation().toRotation2d().plus(Rotation2d.fromDegrees(180))));
        // bottom right
        rightBranches.add(new Pose2d(FieldLayoutConstants.cBranchX,
                FieldLayoutConstants.cBranchY, aprilTagFieldLayout.getTagPose(isBlue?17:8).get()
                        .getRotation().toRotation2d().plus(Rotation2d.fromDegrees(180))));
        // top right
        rightBranches.add(new Pose2d(FieldLayoutConstants.eBranchX,
                FieldLayoutConstants.eBranchY, aprilTagFieldLayout.getTagPose(isBlue?22:9).get()
                        .getRotation().toRotation2d().plus(Rotation2d.fromDegrees(180))));

        // top
        rightBranches.add(new Pose2d(FieldLayoutConstants.gBranchX, FieldLayoutConstants.gBranchY,
                aprilTagFieldLayout.getTagPose(isBlue?21:10).get()
                        .getRotation().toRotation2d().plus(Rotation2d.fromDegrees(180))));
        // top left
        rightBranches.add(new Pose2d(FieldLayoutConstants.rightBranchTopLeftX, FieldLayoutConstants.rightBranchTopLeftY,
                aprilTagFieldLayout.getTagPose(isBlue?20:11).get()
                        .getRotation().toRotation2d().plus(Rotation2d.fromDegrees(180))));
        // bottom left
        rightBranches.add(new Pose2d(FieldLayoutConstants.rightBranchBottomLeftX,
                FieldLayoutConstants.rightBranchBottomLeftY, aprilTagFieldLayout.getTagPose(isBlue?19:6).get()
                        .getRotation().toRotation2d().plus(Rotation2d.fromDegrees(180))));
        return currentPose.nearest(rightBranches);
    }

    public static Pose2d getNearestBranchLeft(Pose2d currentPose) {

        ArrayList<Pose2d> rightBranches = new ArrayList<>();

        // bottom side
        // rightBranches.add(new Pose2d(FieldLayoutConstants.leftBranchBottomX, FieldLayoutConstants.leftBranchBottomY,
        //         aprilTagFieldLayout.getTagPose(isBlue?18:7).get()
        //                 .getRotation().toRotation2d().plus(Rotation2d.fromDegrees(180))));

        // bottom right
        rightBranches.add(new Pose2d(FieldLayoutConstants.leftBranchBottomRightX,
                FieldLayoutConstants.leftBranchBottomRightY, aprilTagFieldLayout.getTagPose(isBlue?17:8).get()
                        .getRotation().toRotation2d().plus(Rotation2d.fromDegrees(180))));

        // // top right
        // rightBranches.add(new Pose2d(FieldLayoutConstants.leftBranchTopRightX, FieldLayoutConstants.leftBranchTopRightY,
        //         aprilTagFieldLayout.getTagPose(isBlue?22:9).get()
        //                 .getRotation().toRotation2d().plus(Rotation2d.fromDegrees(180))));

        // // top
        // rightBranches.add(new Pose2d(FieldLayoutConstants.leftBranchTopX, FieldLayoutConstants.leftBranchTopY,
        //         aprilTagFieldLayout.getTagPose(isBlue?21:10).get()
        //                 .getRotation().toRotation2d().plus(Rotation2d.fromDegrees(180))));
        // // top left 
        // rightBranches.add(new Pose2d(FieldLayoutConstants.leftBranchTopLeftX, FieldLayoutConstants.leftBranchTopLeftY,
        //         aprilTagFieldLayout.getTagPose(isBlue?20:11).get()
        //                 .getRotation().toRotation2d().plus(Rotation2d.fromDegrees(180))));

        // bottom left//
        rightBranches.add(new Pose2d(FieldLayoutConstants.leftBranchBottomLeftX,
                FieldLayoutConstants.leftBranchBottomLeftY, aprilTagFieldLayout.getTagPose(isBlue?19:6).get()
                        .getRotation().toRotation2d().plus(Rotation2d.fromDegrees(180))));
        return currentPose.nearest(rightBranches);

    }
}
