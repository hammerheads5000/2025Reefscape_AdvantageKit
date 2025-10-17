// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static frc.robot.Constants.FieldConstants.REEF_CENTER_BLUE;
import static frc.robot.Constants.FieldConstants.REEF_CENTER_RED;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.ConstraintsZone;
import com.pathplanner.lib.path.EventMarker;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.IdealStartingState;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PointTowardsZone;
import com.pathplanner.lib.path.RotationTarget;
import com.pathplanner.lib.util.FlippingUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.PathConstants;
import java.util.ArrayList;
import java.util.List;

/** Container pathfinding class. */
public class Pathfinding {
    /** Get the closest reef side to the given pose (0-5 starting at A/B going clockwise) */
    public static int getClosestReefSide(Pose2d pose) {
        ArrayList<Pose2d> reefPoses = new ArrayList<>();
        for (int side = 0; side < 6; side++) reefPoses.add(AlignToReefCommands.getReefPose(side, 0));

        return reefPoses.indexOf(pose.nearest(reefPoses));
    }

    /** Get distance betweeen two sides in terms of number of sides (e.g. distance between sides 1 and 3 is 2) */
    private static int distanceBetweenSides(int side1, int side2) {
        int diff = Math.abs(side1 - side2);
        return Math.min(diff, 6 - diff);
    }

    /** Returns a pose pointed toward a translation (general utility method) */
    public static Pose2d pointPoseTowards(Pose2d pose, Translation2d other) {
        Translation2d translation = other.minus(pose.getTranslation());
        return new Pose2d(pose.getTranslation(), translation.getAngle());
    }

    /** Returns a pose pointed toward another pose (general utility method) */
    public static Pose2d pointPoseTowards(Pose2d pose, Pose2d other) {
        return pointPoseTowards(pose, other.getTranslation());
    }

    /** Returns the next side to approach based on the current side and target side */
    private static int getNextSide(int currentSide, int targetSide) {
        int positiveDistance = distanceBetweenSides(currentSide + 1, targetSide);
        int negativeDistance = distanceBetweenSides(currentSide - 1, targetSide);

        if (positiveDistance < negativeDistance) return (currentSide + 1 + 6) % 6;
        return (currentSide - 1 + 6) % 6;
    }

    // both follow reef side order (0 for facing drivers, increasing CW)
    private static final Pose2d[] CCW_APPROACH_POSES = new Pose2d[6];
    private static final Pose2d[] CW_APPROACH_POSES = new Pose2d[6];

    static {
        for (int side = 0; side < 6; side++) {
            Pose2d sidePose = AlignToReefCommands.getReefPose(side, 0);

            // move side pose outwards by TRAVERSE_DISTANCE
            sidePose = sidePose.transformBy(new Transform2d(
                    new Translation2d(PathConstants.TRAVERSE_DISTANCE.unaryMinus(), Meters.zero()), Rotation2d.kZero));

            // give pose tangential rotation for smooth CCW path
            sidePose = sidePose.rotateAround(sidePose.getTranslation(), Rotation2d.kCW_90deg);

            CCW_APPROACH_POSES[side] = sidePose;
            CW_APPROACH_POSES[side] = sidePose.rotateAround(sidePose.getTranslation(), Rotation2d.k180deg);
        }
    }

    /** Generates a list of poses to approach the reef from the current pose to the target side */
    private static ArrayList<Pose2d> generateApproachPoses(Pose2d currentPose, int side) {
        int currentSide = getClosestReefSide(currentPose);

        boolean CW = getNextSide(currentSide, side) == (currentSide + 1) % 6;
        ArrayList<Pose2d> poses = new ArrayList<>();
        // move around reef until within 1 side of target side
        while (distanceBetweenSides(currentSide, side) > 1) {
            int nextSide = getNextSide(currentSide, side);

            poses.add(CW ? CW_APPROACH_POSES[nextSide] : CCW_APPROACH_POSES[nextSide]);
            currentSide = nextSide;
        }

        return poses;
    }

    /**
     * Generates a PathPlannerPath to go to given position
     *
     * @param currentPose Current robot pose
     * @param side Side of the reef to approach (0-5, starting at A/B going clockwise)
     * @param relativePos Relative position on the reef (-1 for right branch, 0 for center, 1 for left branch)
     * @param startSpeeds Initial chassis speeds to use for the path
     */
    public static PathPlannerPath generateReefPath(
            Pose2d currentPose, int side, double relativePos, ChassisSpeeds startSpeeds) {

        ArrayList<Pose2d> poses = generateApproachPoses(currentPose, side);
        Pose2d endPose = AlignToReefCommands.getReefPose(side, relativePos);

        // offset away from reef for straight approach
        Pose2d approachEndPose = AlignToReefCommands.getReefPose(side, relativePos);
        Transform2d shiftApproachTransform = new Transform2d(
                new Translation2d(PathConstants.APPROACH_DISTANCE.unaryMinus(), Meters.zero()), Rotation2d.kZero);
        approachEndPose = approachEndPose.transformBy(shiftApproachTransform);

        poses.add(approachEndPose);
        // poses.add(endPose);

        // turn velocity into translation to determine magnitude
        // Translation2d vel = new Translation2d(startSpeeds.vxMetersPerSecond, startSpeeds.vyMetersPerSecond);

        // if robot is moving fast enough (and in teleop), smoothly transition into path
        // if (DriverStation.isAutonomous() || vel.getNorm() < PathConstants.MIN_PATH_SPEED.in(MetersPerSecond)) {
        poses.add(0, pointPoseTowards(currentPose, poses.get(0)));
        // } else {
        //     poses.add(0, new Pose2d(currentPose.getTranslation(), chassisSpeedsToHeading(startSpeeds)));
        // }

        // add rotation target for end pose
        ArrayList<RotationTarget> rotationTargets = new ArrayList<>();
        rotationTargets.add(new RotationTarget(poses.size() - 2, endPose.getRotation()));

        // point towards the center of the reef
        ArrayList<PointTowardsZone> pointTowardsZones = new ArrayList<>();
        Translation2d reefCenter = AutoBuilder.shouldFlip() ? REEF_CENTER_RED : REEF_CENTER_BLUE;
        pointTowardsZones.add(new PointTowardsZone("Point At Reef", reefCenter, 0, poses.size() - 2));

        ArrayList<ConstraintsZone> constraintsZones = new ArrayList<>();

        // constraint zones to go fast for a proporiton of path, and slow down at the end
        constraintsZones.add(new ConstraintsZone(0, PathConstants.FAST_PROPORTION, PathConstants.FAST_CONSTRAINTS));
        constraintsZones.add(new ConstraintsZone(
                poses.size() - 1 - PathConstants.APPROACH_PROPORTION,
                poses.size() - 1,
                PathConstants.APPROACH_CONSTRAINTS));

        PathPlannerPath path = new PathPlannerPath(
                PathPlannerPath.waypointsFromPoses(poses),
                rotationTargets,
                pointTowardsZones,
                constraintsZones,
                new ArrayList<EventMarker>(),
                PathConstants.CONSTRAINTS,
                new IdealStartingState(chassisSpeedsToVelocity(startSpeeds), currentPose.getRotation()),
                new GoalEndState(0, endPose.getRotation()),
                false);

        path.preventFlipping = true;

        return path;
    }

    /**
     * Generates a PathPlannerPath to go to given position (without initial speeds)
     *
     * @param currentPose Current robot pose
     * @param side Side of the reef to approach (0-5, starting at A/B going clockwise)
     * @param relativePos Relative position on the reef (-1 for right branch, 0 for center, 1 for left branch)
     */
    public static PathPlannerPath generateReefPath(Pose2d currentPose, int side, double relativePos) {
        return generateReefPath(currentPose, side, relativePos, new ChassisSpeeds());
    }

    private static Rotation2d chassisSpeedsToHeading(ChassisSpeeds chassisSpeeds) {
        Translation2d translation = new Translation2d(chassisSpeeds.vxMetersPerSecond, chassisSpeeds.vyMetersPerSecond);
        return translation.getAngle();
    }

    private static LinearVelocity chassisSpeedsToVelocity(ChassisSpeeds chassisSpeeds) {
        Translation2d translation = new Translation2d(chassisSpeeds.vxMetersPerSecond, chassisSpeeds.vyMetersPerSecond);
        return MetersPerSecond.of(translation.getNorm());
    }

    /**
     * Generates a PathPlannerPath to search for coral
     *
     * @param currentPose Current robot pose
     * @param pos Pos number (0 for right, 1 for left)
     * @param startSpeeds Initial chassis speeds to use for the path
     */
    public static PathPlannerPath generateCoralSearchPath(Pose2d currentPose, int pos, ChassisSpeeds startSpeeds) {
        int side = pos == 1 ? 1 : 5;
        ArrayList<Pose2d> poses = generateApproachPoses(currentPose, side);
        Pose2d endPose = pos == 1 ? FieldConstants.LEFT_CORAL_SEARCH_POSE : FieldConstants.RIGHT_CORAL_SEARCH_POSE;
        if (AutoBuilder.shouldFlip()) {
            endPose = FlippingUtil.flipFieldPose(endPose);
        }
        poses.add(new Pose2d(endPose.getTranslation(), endPose.getRotation().plus(Rotation2d.k180deg)));

        Translation2d vel = new Translation2d(startSpeeds.vxMetersPerSecond, startSpeeds.vyMetersPerSecond);

        Pose2d startPose;
        if (DriverStation.isAutonomous() || vel.getNorm() < PathConstants.MIN_PATH_SPEED.in(MetersPerSecond)) {
            startPose = pointPoseTowards(currentPose, poses.get(0));
        } else {
            Rotation2d startHeading = chassisSpeedsToHeading(startSpeeds);
            startPose = new Pose2d(currentPose.getTranslation(), startHeading);
        }
        poses.add(0, startPose);

        List<PointTowardsZone> pointTowardsZones = List.of(new PointTowardsZone(
                "End Rotation Early", endPose.getTranslation(), Rotation2d.k180deg, 1.5, poses.size() - 1));

        PathPlannerPath path = new PathPlannerPath(
                PathPlannerPath.waypointsFromPoses(poses),
                new ArrayList<RotationTarget>(),
                pointTowardsZones,
                new ArrayList<ConstraintsZone>(),
                new ArrayList<EventMarker>(),
                PathConstants.FAST_CONSTRAINTS,
                new IdealStartingState(chassisSpeedsToVelocity(startSpeeds), currentPose.getRotation()),
                new GoalEndState(0, endPose.getRotation()),
                false);

        path.preventFlipping = true;

        return path;
    }

    /**
     * Generates a PathPlannerPath to search for coral (without initial speeds)
     *
     * @param currentPose Current robot pose
     * @param pos Pos number (0 for right, 1 for left)
     */
    public static PathPlannerPath generateCoralSearchPath(Pose2d currentPose, int pos) {
        return generateCoralSearchPath(currentPose, pos, new ChassisSpeeds());
    }

    /**
     * Generates a PathPlannerPath to approach a barge
     *
     * @param currentPose Current robot pose
     * @param pos Barge position (F,G,H,I for barge positions from right to left)
     * @param startSpeeds Initial chassis speeds to use for the path
     */
    public static PathPlannerPath generateBargePath(Pose2d currentPose, char pos, ChassisSpeeds startSpeeds) {
        int side = 2;
        ArrayList<Pose2d> poses = generateApproachPoses(currentPose, side);
        Pose2d endPose = ApproachBargeCommands.getBargePose(pos);
        poses.add(
                new Pose2d(endPose.getTranslation(), AutoBuilder.shouldFlip() ? Rotation2d.k180deg : Rotation2d.kZero));

        Translation2d vel = new Translation2d(startSpeeds.vxMetersPerSecond, startSpeeds.vyMetersPerSecond);

        Pose2d startPose;
        if (DriverStation.isAutonomous() || vel.getNorm() < PathConstants.MIN_PATH_SPEED.in(MetersPerSecond)) {
            startPose = pointPoseTowards(currentPose, poses.get(0));
        } else {
            Rotation2d startHeading = chassisSpeedsToHeading(startSpeeds);
            startPose = new Pose2d(currentPose.getTranslation(), startHeading);
        }
        poses.add(0, startPose);

        ArrayList<ConstraintsZone> constraintsZones = new ArrayList<>();
        constraintsZones.add(
                new ConstraintsZone(poses.size() - 1.3, poses.size() - 1, PathConstants.APPROACH_CONSTRAINTS));

        PathPlannerPath path = new PathPlannerPath(
                PathPlannerPath.waypointsFromPoses(poses),
                new ArrayList<RotationTarget>(),
                new ArrayList<PointTowardsZone>(),
                constraintsZones,
                new ArrayList<EventMarker>(),
                PathConstants.FAST_CONSTRAINTS,
                new IdealStartingState(chassisSpeedsToVelocity(startSpeeds), currentPose.getRotation()),
                new GoalEndState(0, endPose.getRotation()),
                false);

        path.preventFlipping = true;

        return path;
    }

    /**
     * Generates a PathPlannerPath to approach a barge
     *
     * @param currentPose Current robot pose
     * @param pos Barge position (F,G,H,I for barge positions from right to left)
     * @param startSpeeds Initial chassis speeds to use for the path
     */
    public static PathPlannerPath generateProcessPath(Pose2d currentPose, ChassisSpeeds startSpeeds) {
        int side = 4;
        ArrayList<Pose2d> poses = generateApproachPoses(currentPose, side);
        Pose2d endPose = FieldConstants.PROCESSOR;
        if (AutoBuilder.shouldFlip()) {
            endPose = FlippingUtil.flipFieldPose(endPose);
        }
        poses.add(endPose);

        Translation2d vel = new Translation2d(startSpeeds.vxMetersPerSecond, startSpeeds.vyMetersPerSecond);

        Pose2d startPose;
        if (DriverStation.isAutonomous() || vel.getNorm() < PathConstants.MIN_PATH_SPEED.in(MetersPerSecond)) {
            startPose = pointPoseTowards(currentPose, poses.get(0));
        } else {
            Rotation2d startHeading = chassisSpeedsToHeading(startSpeeds);
            startPose = new Pose2d(currentPose.getTranslation(), startHeading);
        }
        poses.add(0, startPose);

        ArrayList<ConstraintsZone> constraintsZones = new ArrayList<>();
        constraintsZones.add(
                new ConstraintsZone(poses.size() - 1.3, poses.size() - 1, PathConstants.APPROACH_CONSTRAINTS));

        PathPlannerPath path = new PathPlannerPath(
                PathPlannerPath.waypointsFromPoses(poses),
                new ArrayList<RotationTarget>(),
                new ArrayList<PointTowardsZone>(),
                constraintsZones,
                new ArrayList<EventMarker>(),
                PathConstants.FAST_CONSTRAINTS,
                new IdealStartingState(chassisSpeedsToVelocity(startSpeeds), currentPose.getRotation()),
                new GoalEndState(0, endPose.getRotation()),
                false);

        path.preventFlipping = true;

        return path;
    }
}
