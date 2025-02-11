// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Util;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import java.util.ArrayList;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.subsystems.ChassisSubsystem;

/** Add your docs here. */
public final class FieldLayout {
    public static AprilTagFieldLayout aprilTagFieldLayout = AprilTagFieldLayout
            .loadField(AprilTagFields.k2025Reefscape);

    public enum ReefSide {
        bottom,
        bottomLeft,
        topLeft,
        top,
        topRight,
        bottomRight
    };

    private FieldLayout() {
    }

    private static Pose2d getReefsSidePose(int ID, boolean right, Distance distance) {
        Pose2d tagPose = aprilTagFieldLayout.getTagPose(ID).get().toPose2d();
        tagPose = new Pose2d(tagPose.getTranslation(), tagPose.getRotation().plus(Rotation2d.fromDegrees(180)));
        return new Pose2d(
                tagPose.getTranslation()
                        .plus(new Translation2d(distance.in(Meters), Rotation2d.fromDegrees(right ? -90 : 90))),
                tagPose.getRotation());
    }

    //TODO: return the driver station allience to the normal code without a parameter
    public static Pose2d getBranchPose(ReefSide reefSide, boolean right,  Distance distance,DriverStation.Alliance alliance) {
        if (alliance == DriverStation.Alliance.Blue) {
            switch (reefSide) {
                case bottom:
                    return getReefsSidePose(18, right, distance);
                case bottomLeft:
                    return getReefsSidePose(19, right, distance);
                case topLeft:
                    return getReefsSidePose(20, right, distance);
                case top:
                    return getReefsSidePose(21, right, distance);
                case topRight:
                    return getReefsSidePose(22, right, distance);
                case bottomRight:
                    return getReefsSidePose(17, right, distance);
            }
        }

        switch (reefSide) {
            case bottom:
                return getReefsSidePose(7, right, distance);
            case bottomLeft:
                return getReefsSidePose(6, right, distance);
            case topLeft:
                return getReefsSidePose(11, right, distance);
            case top:
                return getReefsSidePose(10, right, distance);
            case topRight:
                return getReefsSidePose(9, right, distance);
            case bottomRight:
                return getReefsSidePose(8, right, distance);
            default:
                return new Pose2d();
        }
    }

    private static Pose2d getCoralSourcePose2d(int ID, Distance distance) {
        Pose2d tagPos = aprilTagFieldLayout.getTagPose(ID).get().toPose2d();
        return new Pose2d(tagPos.getTranslation().plus(new Translation2d(distance.in(Meters), tagPos.getRotation())),
                tagPos.getRotation().plus(Rotation2d.fromDegrees(180)));
    }

    //TODO: return the driver station allience to the normal code without a parameter
    public static Pose2d getCoralSourcePose(Distance distance,Pose2d currentPose2d,DriverStation.Alliance alliance) {
        ArrayList<Pose2d> coralSourceArrayList = new ArrayList<>();
        
        coralSourceArrayList.add(getCoralSourcePose2d(alliance == DriverStation.Alliance.Blue ? 12 : 2, distance));
        coralSourceArrayList.add(getCoralSourcePose2d(alliance == DriverStation.Alliance.Blue ? 13 : 1, distance));
        // @Tal: Nearest to... what..?
        return currentPose2d.nearest(coralSourceArrayList);

    }
    // @Tal: use the Alliance enum
    public static Pose2d getProccesorPose2d(Distance distance) {
        
        // @Tal: Im pretty sure its +90 in both cases
        Pose2d tagPos = aprilTagFieldLayout.getTagPose(DriverStation.getAlliance().get() == DriverStation.Alliance.Blue ? 16 : 3).get().toPose2d();
        return new Pose2d(tagPos.getTranslation().plus(new Translation2d(distance.in(Meters), tagPos.getRotation())),
                tagPos.getRotation().plus(Rotation2d.fromDegrees( 90)));
    }
}

// var p = x.getTagPose(10).get().toPose2d();
// p = new Pose2d(p.getTranslation(),
// p.getRotation().plus(Rotation2d.fromDegrees(180)));
// p = new Pose2d(p.getTranslation().minus(new Translation2d(0.5,
// p.getRotation())), p.getRotation());
// var left = new Pose2d(p.getTranslation().plus(new
// Translation2d(Inches.of(6.5).in(Meters),
// p.getRotation().plus(Rotation2d.fromDegrees(90)))), p.getRotation());
// var right = new Pose2d(p.getTranslation().plus(new
// Translation2d(Inches.of(6.5).in(Meters),
// p.getRotation().plus(Rotation2d.fromDegrees(-90)))), p.getRotation());