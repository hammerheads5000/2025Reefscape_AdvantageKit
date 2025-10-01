// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static edu.wpi.first.units.Units.Meters;

import com.pathplanner.lib.util.FlippingUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants.AlignConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.PathConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.vision.Vision;
import java.util.Optional;
import org.littletonrobotics.junction.Logger;

/** Container class for aligning to reef */
public class AlignToReefCommands {
    private static final Pose2d[] RED_REEF_POSES = new Pose2d[6];
    private static final Pose2d[] BLUE_REEF_POSES = new Pose2d[6];

    private static final Pose2d[] RED_BRANCH_POSES = new Pose2d[6];
    private static final Pose2d[] BLUE_BRANCH_POSES = new Pose2d[6];

    // pregenerate reef and branch poses
    static {
        for (int side = 0; side < 6; side++) {
            // initially do all calculations from blue, then flip later
            Translation2d reefCenter = FieldConstants.REEF_CENTER_BLUE;

            // robot position centered on close reef side
            Translation2d translation =
                    reefCenter.plus(new Translation2d(FieldConstants.REEF_APOTHEM.unaryMinus(), Meters.zero()));

            // rotate to correct side
            translation = translation.rotateAround(reefCenter, Rotation2d.fromDegrees(-60 * side));

            // make pose from translation and correct rotation
            Pose2d reefPose = new Pose2d(translation, Rotation2d.fromDegrees(-60 * side));

            BLUE_REEF_POSES[side] = reefPose;
            RED_REEF_POSES[side] = FlippingUtil.flipFieldPose(reefPose);

            Transform2d branchTransform = new Transform2d(
                    new Translation2d(PathConstants.DISTANCE_TO_REEF.plus(PathConstants.BRANCH_INSET), Meters.zero()),
                    Rotation2d.kZero);

            BLUE_BRANCH_POSES[side] = BLUE_REEF_POSES[side].plus(branchTransform);
            RED_BRANCH_POSES[side] = RED_REEF_POSES[side].plus(branchTransform);
        }
    }

    /**
     * Calculates the pose of the robot for scoring on a branch or trough.
     *
     * @param side The side of the reef (0 for left, increases clockwise).
     * @param relativePos The relative position on the reef (-1 for right branch, 0 for center, 1 for left branch).
     * @return The calculated Pose2d for scoring.
     */
    public static Pose2d getReefPose(int side, double relativePos) {
        // determine whether to use red or blue reef position
        boolean isRed = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red;

        return getReefPose(side, relativePos, isRed);
    }

    /**
     * Calculates the pose of the robot for scoring on a branch or trough.
     *
     * @param side The side of the reef (0 for left, increases clockwise).
     * @param relativePos The relative position on the reef (-1 for right branch, 0 for center, 1 for left branch).
     * @return The calculated Pose2d for scoring.
     */
    public static Pose2d getReefPose(int side, double relativePos, boolean isRed) {
        Transform2d branchTransform =
                new Transform2d(FieldConstants.CENTERED_TO_LEFT_BRANCH.times(relativePos), Rotation2d.kZero);
        return isRed ? RED_REEF_POSES[side].plus(branchTransform) : BLUE_REEF_POSES[side].plus(branchTransform);
    }

    /**
     * Calculates the pose of a branch.
     *
     * @param side The side of the reef (0 for left, increases clockwise).
     * @param relativePos The relative position on the reef (-1 for right branch, 0 for center, 1 for left branch).
     * @return The calculated Pose2d for scoring.
     */
    public static Pose2d getBranchPose(int side, double relativePos) {
        // determine whether to use red or blue reef position
        boolean isRed = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red;

        return getBranchPose(side, relativePos, isRed);
    }

    /**
     * Calculates the pose of a branch.
     *
     * @param side The side of the reef (0 for left, increases clockwise).
     * @param relativePos The relative position on the reef (-1 for right branch, 0 for center, 1 for left branch).
     * @return The calculated Pose2d for scoring.
     */
    public static Pose2d getBranchPose(int side, double relativePos, boolean isRed) {
        Transform2d branchTransform =
                new Transform2d(FieldConstants.CENTERED_TO_LEFT_BRANCH.times(relativePos), Rotation2d.kZero);
        return isRed ? RED_BRANCH_POSES[side].plus(branchTransform) : BLUE_BRANCH_POSES[side].plus(branchTransform);
    }

    public static AlignToPoseCommand alignToReef(int side, double relativePos, Swerve swerve) {
        return new AlignToPoseCommand(
                getReefPose(side, relativePos),
                AlignConstants.SCORING_PID_TRANSLATION,
                AlignConstants.SCORING_PID_ANGLE,
                swerve);
    }

    public static AlignAndFacePoseCommand alignToReefFacingBranch(int side, double relativePos, Swerve swerve) {
        return new AlignAndFacePoseCommand(
                getReefPose(side, relativePos),
                getBranchPose(side, relativePos),
                AlignConstants.SCORING_PID_TRANSLATION,
                AlignConstants.SCORING_PID_ANGLE,
                swerve);
    }

    public static AlignAndFacePoseCommand advancedAlignToReef(
            int side, double relativePos, Swerve swerve, Vision vision, Elevator elevator) {
        return new AlignAndFacePoseCommand(
                getReefPose(side, relativePos),
                getBranchPose(side, relativePos),
                () -> processBranchPos(side, relativePos, swerve, vision, elevator),
                AlignConstants.SCORING_PID_TRANSLATION,
                AlignConstants.SCORING_PID_ANGLE,
                swerve);
    }

    private static Pose2d processBranchPos(
            int side, double relativePos, Swerve swerve, Vision vision, Elevator elevator) {
        Optional<Pose2d> branchPosOptional = vision.getBranchPos();
        if (branchPosOptional.isEmpty()) {
            Logger.recordOutput("ReefVision/Tracking", "no target");
            return getBranchPose(side, relativePos);
        }
        if (elevator.getHeight().lte(VisionConstants.MIN_HEIGHT_FOR_ACCURACY)) {
            Logger.recordOutput("ReefVision/Tracking", "elevator too low");
            return getBranchPose(side, relativePos);
        }

        Pose2d branchPos = branchPosOptional.get();

        if (branchPos
                        .getTranslation()
                        .getDistance(getBranchPose(side, relativePos).getTranslation())
                > VisionConstants.MAX_DISTANCE_TO_BRANCH.in(Meters)) {
            Logger.recordOutput("ReefVision/Tracking", "too far from ideal");
            return getBranchPose(side, relativePos);
        }

        Logger.recordOutput("ReefVision/Tracking", "tracking");
        return branchPos;
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
            poses[side] = getReefPose(side, relativePos, isRed);
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

        Pose2d[] poses = new Pose2d[6];
        for (int side = 0; side < 6; side++) {
            poses[side] = getBranchPose(side, relativePos, isRed);
        }

        Logger.recordOutput(topicName, poses);
    }
}
