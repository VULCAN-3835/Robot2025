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
import frc.robot.Constants.ChassisConstants.distanceConstants;

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

    public static Pose2d getBranchPose(ReefSide reefSide, boolean right,  Distance distance) {
        boolean isBlue = DriverStation.getAlliance().get() == DriverStation.Alliance.Blue;
            switch (reefSide) {
                case bottom:
                    return getReefsSidePose(isBlue?18:7, right, distance);
                case bottomLeft:
                    return getReefsSidePose(isBlue?19:6, right, distance);
                case topLeft:
                    return getReefsSidePose(isBlue?20:11, right, distance);
                case top:
                    return getReefsSidePose(isBlue?21:10, right, distance);
                case topRight:
                    return getReefsSidePose(isBlue?22:9, right, distance);
                case bottomRight:
                    return getReefsSidePose(isBlue?17:8, right, distance);
                default:
                    System.out.println("error in get Branch Pose");
                    return new Pose2d();
            }
    }

    private static Pose2d getCoralSourcePose2d(int ID, Distance distance) {
        Pose2d tagPos = aprilTagFieldLayout.getTagPose(ID).get().toPose2d();
        return new Pose2d(tagPos.getTranslation().plus(new Translation2d(distance.in(Meters), tagPos.getRotation())),
                tagPos.getRotation().plus(Rotation2d.fromDegrees(180)));
    }

    public static Pose2d getCoralSourcePose(Distance distance,Pose2d currentPose2d) {
        ArrayList<Pose2d> coralSourceArrayList = new ArrayList<>();
        
        coralSourceArrayList.add(getCoralSourcePose2d(DriverStation.getAlliance().get() == DriverStation.Alliance.Blue ? 12 : 2, distance));
        coralSourceArrayList.add(getCoralSourcePose2d(DriverStation.getAlliance().get() == DriverStation.Alliance.Blue ? 13 : 1, distance));
        return currentPose2d.nearest(coralSourceArrayList);

    }
    public static Pose2d getProccesorPose2d(Distance distance) {        
        Pose2d tagPos = aprilTagFieldLayout.getTagPose(DriverStation.getAlliance().get() == DriverStation.Alliance.Blue ? 16 : 3).get().toPose2d();
        return new Pose2d(tagPos.getTranslation().plus(new Translation2d(distance.in(Meters), tagPos.getRotation())),
                tagPos.getRotation().plus(Rotation2d.fromDegrees( 90)));
    }

    public static Pose2d getBranchMid(ReefSide reefSide,Distance distance){
        boolean isBlue = DriverStation.getAlliance().get()==DriverStation.Alliance.Blue;
        switch (reefSide) {
            case top:
                return getCoralSourcePose2d(isBlue?21:10, distance);
            case topRight:
                return getCoralSourcePose2d(isBlue?22:9, distance);
            case topLeft:
                return getCoralSourcePose2d(isBlue?20:11, distance);
            case bottom:
                return getCoralSourcePose2d(isBlue?18:7, distance);
            case bottomLeft:
                return getCoralSourcePose2d(isBlue?19:6, distance);
            case bottomRight:
                return getCoralSourcePose2d(isBlue?17:8, distance);
            default:
                return new Pose2d();
        }      
          
    }

    public static Pose2d getNearestBranch(Pose2d currentPose2d,boolean right){
        ArrayList<Pose2d> list = new ArrayList<>();
        list.add(getBranchPose(ReefSide.top,right,distanceConstants.topReefDistance));
        list.add(getBranchPose(ReefSide.topRight,right,distanceConstants.topRightReefDistance));
        list.add(getBranchPose(ReefSide.topLeft,right,distanceConstants.topLeftReefDistance));
        list.add(getBranchPose(ReefSide.top,right,distanceConstants.bottomReefDistance));
        list.add(getBranchPose(ReefSide.topRight,right,distanceConstants.bottomRightReefDistance));
        list.add(getBranchPose(ReefSide.topLeft,right,distanceConstants.bottomLeftReefDistance));
        return currentPose2d.nearest(list);
    }
}

