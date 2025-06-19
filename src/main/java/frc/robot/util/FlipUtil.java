// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.FieldConstants;

/** Utility class for flipping positions and poses based on the robot's alliance. */
public class FlipUtil {
    public static Translation2d flip(Translation2d pos) {
        return new Translation2d(FieldConstants.FIELD_LENGTH - pos.getX(), FieldConstants.FIELD_WIDTH - pos.getY());
    }

    public static Pose2d flip(Pose2d pose) {
        return new Pose2d(flip(pose.getTranslation()), pose.getRotation().rotateBy(Rotation2d.k180deg));
    }

    public static Translation3d flip(Translation3d pos) {
        return new Translation3d(
                FieldConstants.FIELD_LENGTH - pos.getX(), FieldConstants.FIELD_WIDTH - pos.getY(), pos.getZ());
    }

    public static Pose3d flip(Pose3d pose) {
        return new Pose3d(flip(pose.getTranslation()), pose.getRotation().rotateBy(new Rotation3d(0, 0, Math.PI)));
    }

    public static boolean shouldFlip() {
        return DriverStation.getAlliance().isPresent()
                && DriverStation.getAlliance().get() == DriverStation.Alliance.Red;
    }

    public static Translation2d applyAlliance(Translation2d pos) {
        return shouldFlip() ? flip(pos) : pos;
    }

    public static Pose2d applyAlliance(Pose2d pose) {
        return shouldFlip() ? flip(pose) : pose;
    }

    public static Translation3d applyAlliance(Translation3d pos) {
        return shouldFlip() ? flip(pos) : pos;
    }

    public static Pose3d applyAlliance(Pose3d pose) {
        return shouldFlip() ? flip(pose) : pose;
    }
}
