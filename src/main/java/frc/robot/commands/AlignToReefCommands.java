// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Constants.AlignConstants;
import frc.robot.FieldConstants.Reef;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.util.FlipUtil;
import org.littletonrobotics.junction.Logger;

/** Container class for aligning to reef */
public class AlignToReefCommands {
    public static AlignToPoseCommand alignToReef(int side, double relativePos, Swerve swerve) {
        return new AlignToPoseCommand(
                FlipUtil.applyAlliance(Reef.getReefPose(side, relativePos)),
                AlignConstants.SCORING_PID_TRANSLATION,
                AlignConstants.SCORING_PID_ANGLE,
                swerve);
    }

    public static AlignAndFacePoseCommand alignToReefFacingBranch(int side, double relativePos, Swerve swerve) {
        return new AlignAndFacePoseCommand(
                FlipUtil.applyAlliance(Reef.getReefPose(side, relativePos)),
                FlipUtil.applyAlliance(Reef.getBranchPos(side, relativePos)),
                AlignConstants.SCORING_PID_TRANSLATION,
                AlignConstants.SCORING_PID_ANGLE,
                swerve);
    }

    public static void testReefPoses() {
        testReefPoses(false, -1);
        testReefPoses(false, 0);
        testReefPoses(false, 1);
        testReefPoses(true, -1);
        testReefPoses(true, 0);
        testReefPoses(true, 1);
    }

    private static void testReefPoses(boolean isRed, int relativePos) {
        String topicName = "Reef Alignment Poses/";
        topicName += isRed ? "Red " : "Blue ";
        if (relativePos == -1) {
            topicName += "Right ";
        } else if (relativePos == 1) {
            topicName += "Left ";
        } else {
            topicName += "Center ";
        }

        Pose2d[] poses = new Pose2d[6];
        for (int side = 0; side < 6; side++) {
            poses[side] = Reef.getReefPose(side, relativePos);
            if (isRed) {
                poses[side] = FlipUtil.flip(poses[side]);
            }
        }

        Logger.recordOutput(topicName, poses);
    }

    public static void testBranchPoses() {
        testBranchPoses(false, -1);
        testBranchPoses(false, 0);
        testBranchPoses(false, 1);
        testBranchPoses(true, -1);
        testBranchPoses(true, 0);
        testBranchPoses(true, 1);
    }

    private static void testBranchPoses(boolean isRed, int relativePos) {
        String topicName = "Reef Branch Poses/";
        topicName += isRed ? "Red " : "Blue ";
        if (relativePos == -1) {
            topicName += "Right ";
        } else if (relativePos == 1) {
            topicName += "Left ";
        } else {
            topicName += "Center ";
        }

        Translation2d[] poses = new Translation2d[6];
        for (int side = 0; side < 6; side++) {
            poses[side] = Reef.getBranchPos(side, relativePos);
            if (isRed) {
                poses[side] = FlipUtil.flip(poses[side]);
            }
        }

        Logger.recordOutput(topicName, poses);
    }
}
