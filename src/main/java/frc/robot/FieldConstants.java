// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import frc.robot.Constants.PathConstants;
import frc.robot.util.FlipUtil;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Map;
import org.littletonrobotics.junction.Logger;

/**
 * Side: 0-5 starting at A/B going CCW Branch: 0-11 or A-L (CCW) Station: 0 for right, 1 for left Barge: F-I right to
 * left All measurements in meters
 */
public class FieldConstants {
    public static final AprilTagFieldLayout APRIL_TAGS =
            AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeAndyMark);
    public static final double FIELD_WIDTH = APRIL_TAGS.getFieldWidth(); // 8.042 m (1/2 = 4.021 m)
    public static final double FIELD_LENGTH = APRIL_TAGS.getFieldLength(); // 17.548 m (1/2 = 8.774 m)

    private static Pose2d getTagPose(int id) {
        return APRIL_TAGS.getTagPose(id).get().toPose2d();
    }

    private static Translation2d getTagPos(int id) {
        return APRIL_TAGS.getTagPose(id).get().toPose2d().getTranslation();
    }

    private static Pose2d rotate180(Pose2d pose) {
        return new Pose2d(pose.getTranslation(), pose.getRotation().rotateBy(Rotation2d.k180deg));
    }

    /**
     * Transforms an AprilTag pose to robot pose to approach (facing AprilTag)
     *
     * @param pose
     * @param approachDistance distance away from AprilTag
     * @return
     */
    private static Pose2d approachTransform(Pose2d pose, Distance approachDistance) {
        return approachTransform(pose, approachDistance, Meters.zero());
    }

    /**
     * Transforms an AprilTag pose to robot pose to approach (facing AprilTag)
     *
     * @param pose
     * @param approachDistance distance away from AprilTag
     * @param sideOffset distance to the side (+ -> right, - -> left)
     * @return
     */
    private static Pose2d approachTransform(Pose2d pose, Distance approachDistance, Distance sideOffset) {
        return approachTransform(pose, approachDistance.in(Meters), sideOffset.in(Meters));
    }

    /**
     * Transforms an AprilTag pose to robot pose to approach (facing AprilTag)
     *
     * @param pose
     * @param approachDistance distance away from AprilTag
     * @param sideOffset distance to the side (+ -> right, - -> left)
     * @return
     */
    private static Pose2d approachTransform(Pose2d pose, double approachDistance, double sideOffset) {
        Translation2d translation = new Translation2d(approachDistance, sideOffset);
        return pose.transformBy(new Transform2d(translation, Rotation2d.k180deg));
    }

    public static class Reef {
        public static final Map<Character, Pair<Integer, Integer>> LETTER_TO_SIDE_AND_RELATIVE = Map.ofEntries(
                Map.entry(Character.valueOf('A'), new Pair<Integer, Integer>(0, -1)),
                Map.entry(Character.valueOf('B'), new Pair<Integer, Integer>(0, 1)),
                Map.entry(Character.valueOf('C'), new Pair<Integer, Integer>(1, -1)),
                Map.entry(Character.valueOf('D'), new Pair<Integer, Integer>(1, 1)),
                Map.entry(Character.valueOf('E'), new Pair<Integer, Integer>(2, -1)),
                Map.entry(Character.valueOf('F'), new Pair<Integer, Integer>(2, 1)),
                Map.entry(Character.valueOf('G'), new Pair<Integer, Integer>(3, -1)),
                Map.entry(Character.valueOf('H'), new Pair<Integer, Integer>(3, 1)),
                Map.entry(Character.valueOf('I'), new Pair<Integer, Integer>(4, -1)),
                Map.entry(Character.valueOf('J'), new Pair<Integer, Integer>(4, 1)),
                Map.entry(Character.valueOf('K'), new Pair<Integer, Integer>(5, -1)),
                Map.entry(Character.valueOf('L'), new Pair<Integer, Integer>(5, 1)));

        // Pose at midpoint between tags 18 and 21 (which are opposite on blue reef)
        public static final Translation2d REEF_CENTER =
                getTagPos(18).plus(getTagPos(21)).div(2);

        // Distance from center of robot to center of reef
        // Found by taking distance from tag 18 to center and adding offset from reef
        public static final Distance REEF_APOTHEM = Meters.of(getTagPos(18).getDistance(REEF_CENTER));

        // translation to move from centered on a side to scoring position for the left
        // branch
        public static final Distance BRANCH_OFFSET = Inches.of(12.94 / 2);

        // translation to move to tip of branch
        public static final Translation2d BRANCH_TRANSLATION = new Translation2d(Inches.of(-2), BRANCH_OFFSET);

        public static final Distance L1_OFFSET = Inches.of(16);
        public static final double L1_RELATIVE_POS =
                L1_OFFSET.div(BRANCH_OFFSET).magnitude();

        /**
         * Calculates the pose of the robot for scoring on a branch or trough.
         *
         * @param side The side of the reef (0 for closest to driver station, increases CCW).
         * @param relativePos The relative position on the reef (1 for right branch, 0 for center, -1 for left branch).
         * @return The calculated pose for scoring.
         */
        public static Pose2d getReefPose(int side, double relativePos) {
            return getReefPose(side, relativePos, PathConstants.DISTANCE_TO_REEF);
        }

        /**
         * Calculates the pose of the robot for scoring on a branch or trough.
         *
         * @param side The side of the reef (0 for closest to driver station, increases CCW).
         * @param relativePos The relative position on the reef (1 for right branch, 0 for center, -1 for left branch).
         * @param distance Distance to approach the reef.
         * @return The calculated pose for scoring.
         */
        public static Pose2d getReefPose(int side, double relativePos, Distance distance) {
            Pose2d pose = approachTransform(getTagPose(18), distance, BRANCH_OFFSET.times(relativePos));

            return pose.rotateAround(REEF_CENTER, Rotation2d.fromDegrees(60 * side));
        }

        /**
         * Calculates the position of a branch.
         *
         * @param side The side of the reef (0 for closest to driver station, increases CCW).
         * @param relativePos The relative position on the reef (1 for right branch, 0 for center, -1 for left branch).
         * @return The calculated position to point at.
         */
        public static Translation2d getBranchPos(int side, double relativePos) {
            Translation2d pos = approachTransform(
                            getTagPose(18), BRANCH_TRANSLATION.getX(), BRANCH_TRANSLATION.getY() * relativePos)
                    .getTranslation();

            return pos.rotateAround(REEF_CENTER, Rotation2d.fromDegrees(60 * side));
        }

        private static Map<Integer, Distance> CORAL_HEIGHTS = Map.of(
                1, Inches.of(19),
                2, Inches.of(31.875 - Math.cos(Math.toRadians(-35)) * 0.625),
                3, Inches.of(47.625 - Math.cos(Math.toRadians(-35)) * 0.625),
                4, Inches.of(72));

        private static Map<Integer, Angle> CORAL_ANGLES = Map.of(
                1, Degrees.of(0),
                2, Degrees.of(35),
                3, Degrees.of(35),
                4, Degrees.of(-90));

        /**
         * Calculates the pose of a coral on the reef
         *
         * @param side The side of the reef (0 for closest to driver station, increases CCW).
         * @param relativePos The relative position on the reef (1 for right branch, 0 for center, -1 for left branch).
         * @param level The level of the coral
         * @return
         */
        public static Pose3d getBranchCoralPose(int side, double relativePos, int level) {
            Translation3d pos = new Translation3d(getBranchPos(side, relativePos))
                    .plus(new Translation3d(0, 0, CORAL_HEIGHTS.get(level).in(Meters)));
            Rotation3d rotation = new Rotation3d(Degrees.zero(), CORAL_ANGLES.get(level), Degrees.of(60 * side));
            return new Pose3d(pos, rotation);
        }

        /**
         * Calculates the position of algae on the reef.
         *
         * @param side The side of the reef (0 for closest to driver station, increases CCW).
         * @return
         */
        public static Translation3d getAlgaePose(int side) {
            // use midpoint between L2 and L3 as height
            Translation3d branch1 = getBranchCoralPose(side, 0, 2).getTranslation();
            Translation3d branch2 = getBranchCoralPose(side, 0, 3).getTranslation();

            // adjust whether algae should be high or low
            Translation3d offset =
                    new Translation3d(0.0, 0.0, ((side % 2 == 0) ? branch2.getZ() - branch1.getZ() : 0.0));

            return branch1.interpolate(branch2, 0.5).plus(offset);
        }
    }

    public static final Pose2d STATION_0 = getTagPose(12);
    public static final Pose2d STATION_1 = getTagPose(13);

    /**
     * @param station 0 for right station, 1 for left from operator perspective
     * @param relativePos 1 for right, 0 for center, -1 for left from operator perspective
     * @return Station
     */
    public static Pose2d getStationPose(int station, int relativePos) {
        return getStationPose(
                station, PathConstants.STATION_APPROACH_DISTANCE, PathConstants.SIDE_STATION_OFFSET.times(relativePos));
    }

    /**
     * @param station 0 for right station, 1 for left from operator perspective
     * @param approachDistance distance to approach the station
     * @param sideOffset distance to the side (+ -> right, - -> left)
     * @return Station
     */
    public static Pose2d getStationPose(int station, Distance approachDistance, Distance sideOffset) {
        Pose2d pose = station == 1 ? STATION_1 : STATION_0;
        return approachTransform(pose, approachDistance, sideOffset);
    }

    private static final Rotation2d BARGE_SHOOT_ROTATION = Rotation2d.fromDegrees(-20);
    private static final Distance BARGE_X = Meters.of(7.5);

    private static final Map<Character, Pose2d> BARGE_POSES = Map.of(
            'I', new Pose2d(BARGE_X, Meters.of(7.4), BARGE_SHOOT_ROTATION),
            'H', new Pose2d(BARGE_X, Meters.of(6.7), BARGE_SHOOT_ROTATION),
            'G', new Pose2d(BARGE_X, Meters.of(5.2), BARGE_SHOOT_ROTATION.unaryMinus()),
            'F', new Pose2d(BARGE_X, Meters.of(4.3), BARGE_SHOOT_ROTATION.unaryMinus()));

    private static final Distance LOLLIPOP_X = Inches.of(48);
    public static final Translation2d[] LOLLIPOP_POSES = {
        new Translation2d(LOLLIPOP_X, Inches.of(158.5 - 72)),
        new Translation2d(LOLLIPOP_X, Inches.of(158.5)),
        new Translation2d(LOLLIPOP_X, Inches.of(158.5 + 72))
    };

    public static Pose2d getBargePose(char pos) {
        return BARGE_POSES.get(pos);
    }

    public static final Pose2d PROCESSOR = rotate180(getTagPose(16));

    /**
     * Robot pose to score in processor with distance away from processor
     *
     * @param distance distance to approach the processor
     * @return
     */
    public static final Pose2d getProcessor(Distance distance) {
        return approachTransform(PROCESSOR, distance);
    }

    public static void updateReef(ReefState reefState) {
        ArrayList<Pose3d> corals = new ArrayList<>();
        for (int branch = 0; branch < reefState.coral().length; branch++) {
            for (int level = 0; level < 4; level++) {
                if (reefState.coral()[branch][level]) {
                    double relativePos = (branch % 2 == 0) ? -1 : 1; // even = left, odd = right
                    relativePos *= (level == 0) ? 1.3 : 1;
                    Pose3d coralPose = Reef.getBranchCoralPose(branch / 2, relativePos, level + 1);
                    if (level == 0) {
                        coralPose =
                                coralPose.transformBy(new Transform3d(new Translation3d(0.02, 0, 0), Rotation3d.kZero));
                        coralPose = coralPose.rotateAround(
                                coralPose.getTranslation(),
                                new Rotation3d(
                                        Degrees.zero(), Degrees.zero(), Degrees.of(Math.copySign(90, relativePos))));
                    }
                    corals.add(FlipUtil.applyAlliance(coralPose));
                }
            }
        }
        Logger.recordOutput("Mechanism3d/Reef/Coral", corals.toArray(Pose3d[]::new));
        ArrayList<Translation3d> algae = new ArrayList<>();
        for (int i = 0; i < 6; i++) {
            if (reefState.algae()[i]) {
                algae.add(FlipUtil.applyAlliance(Reef.getAlgaePose(i)));
            }
        }
        Logger.recordOutput("Mechanism3d/Reef/Algae", algae.toArray(Translation3d[]::new));
    }

    /**
     * Way of representing state coral and algae present on reef Coral is represented in a 12x4 boolean array, where
     * each row is a branch Algae is represented in a boolean array of length 6 for each side
     */
    public static record ReefState(boolean[][] coral, boolean[] algae) {
        public ReefState() {
            this(new boolean[12][4], new boolean[6]);
        }

        public ReefState(boolean allCoral, boolean allAlgae) {
            this();
            for (int i = 0; i < coral.length; i++) {
                Arrays.fill(coral[i], allCoral);
            }
            Arrays.fill(algae, allAlgae);
        }

        @Override
        public final String toString() {
            String str = "Coral: ";
            for (boolean[] branch : coral) {
                str += Arrays.toString(branch) + "\n";
            }
            str += "\nAlgae: " + algae;
            return str;
        }

        @Override
        public final boolean equals(Object other) {
            if (this == other) {
                return true;
            }
            if (!(other instanceof ReefState)) {
                return false;
            }
            ReefState otherReef = (ReefState) other;
            return Arrays.deepEquals(coral, otherReef.coral) && Arrays.equals(algae, otherReef.algae);
        }
    }
}
