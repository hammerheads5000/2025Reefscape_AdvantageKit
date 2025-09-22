// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Constants.Dimensions;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.PathConstants;
import frc.robot.Constants.VisionConstants;
import org.littletonrobotics.junction.Logger;

/** Add your docs here. */
public class BoundaryProtections {
    // perimeter, counter clockwise
    private static final double[] PERIMETER_X = {1.758, 15.791, 17.548, 17.548, 15.791, 1.758, 0.0, 0.0};

    private static final double[] PERIMETER_Y = {0.0, 0.0, 1.267, 6.775, 8.052, 8.052, 6.775, 1.267};

    private static final double[] EDGE_VECTORS_X = new double[PERIMETER_X.length];
    private static final double[] EDGE_VECTORS_Y = new double[PERIMETER_X.length];

    // precompute edge vectors
    static {
        double pointBx = PERIMETER_X[PERIMETER_X.length - 1];
        double pointBy = PERIMETER_Y[PERIMETER_Y.length - 1];
        for (int i = 0; i < PERIMETER_X.length; i++) {
            double pointAx = PERIMETER_X[i];
            double pointAy = PERIMETER_Y[i];
            double normSquared = (pointAx - pointBx) * (pointAx - pointBx) + (pointAy - pointBy) * (pointAy - pointBy);
            EDGE_VECTORS_X[i] = (pointAx - pointBx) / normSquared;
            EDGE_VECTORS_Y[i] = (pointAy - pointBy) / normSquared;
            pointBx = pointAx;
            pointBy = pointAy;
        }
    }

    private static final double CIRCUMSCRIBED_REEF_RADIUS =
            FieldConstants.REEF_APOTHEM.minus(PathConstants.DISTANCE_TO_REEF).in(Meters) * 2 / Math.sqrt(3);
    private static final double[] BLUE_REEF_X = new double[6];
    private static final double[] BLUE_REEF_Y = new double[6];
    private static final double[] RED_REEF_X = new double[6];
    private static final double[] RED_REEF_Y = new double[6];

    private static final double[] REEF_EDGE_VECTORS_X = new double[6];
    private static final double[] REEF_EDGE_VECTORS_Y = new double[6];

    // precompute reef vertices and edges
    static {
        double pointBx = BLUE_REEF_X[5];
        double pointBy = BLUE_REEF_Y[5];
        for (int i = 0; i < 6; i++) {
            double angle = Math.PI / 3 * i + Math.PI / 6; // start at 30 degrees
            BLUE_REEF_X[i] = FieldConstants.REEF_CENTER_BLUE.getX() + CIRCUMSCRIBED_REEF_RADIUS * Math.cos(angle);
            BLUE_REEF_Y[i] = FieldConstants.REEF_CENTER_BLUE.getY() + CIRCUMSCRIBED_REEF_RADIUS * Math.sin(angle);
            RED_REEF_X[i] = FieldConstants.REEF_CENTER_RED.getX() + CIRCUMSCRIBED_REEF_RADIUS * Math.cos(angle);
            RED_REEF_Y[i] = FieldConstants.REEF_CENTER_RED.getY() + CIRCUMSCRIBED_REEF_RADIUS * Math.sin(angle);

            double pointAx = BLUE_REEF_X[i];
            double pointAy = BLUE_REEF_Y[i];
            double normSquared = (pointAx - pointBx) * (pointAx - pointBx) + (pointAy - pointBy) * (pointAy - pointBy);
            REEF_EDGE_VECTORS_X[i] = (pointAx - pointBx) / normSquared;
            REEF_EDGE_VECTORS_Y[i] = (pointAy - pointBy) / normSquared;
            pointBx = pointAx;
            pointBy = pointAy;
        }
    }

    // nearest point on perimeter, facing inwards
    public static final Pose2d nearestPerimeterPose(Translation2d pos) {
        final double x = pos.getX();
        final double y = pos.getY();

        double nearestNormSquared = Double.POSITIVE_INFINITY;
        double nearestX = 0, nearestY = 0;
        double nearestEdgeVecX = 0, nearestEdgeVecY = 0;

        double pointAx, pointAy;
        double pointBx = PERIMETER_X[PERIMETER_X.length - 1];
        double pointBy = PERIMETER_Y[PERIMETER_Y.length - 1];
        for (int i = 0; i < PERIMETER_X.length; i++) { // loop over edges
            pointAx = pointBx;
            pointAy = pointBy;

            pointBx = PERIMETER_X[i];
            pointBy = PERIMETER_Y[i];

            double t = (x - pointAx) * EDGE_VECTORS_X[i]
                    + (y - pointAy) * EDGE_VECTORS_Y[i]; // dot product (pos - A) * edge vector
            t = MathUtil.clamp(t, 0, 1);

            // closest point on this edge
            double possNearestX = (1 - t) * pointAx + t * pointBx;
            double possNearestY = (1 - t) * pointAy + t * pointBy;

            double possNormSquared = (x - possNearestX) * (x - possNearestX) + (y - possNearestY) * (y - possNearestY);

            if (possNormSquared < nearestNormSquared) {
                nearestNormSquared = possNormSquared;
                nearestX = possNearestX;
                nearestY = possNearestY;
                nearestEdgeVecX = EDGE_VECTORS_X[i];
                nearestEdgeVecY = EDGE_VECTORS_Y[i];
            }
        }

        // form pose from point and rotation from rotated edge vector
        return new Pose2d(nearestX, nearestY, new Rotation2d(-nearestEdgeVecY, nearestEdgeVecX));
    }

    // nearest point on reef, facing outwards
    public static final Pose2d nearestReefPose(Translation2d pos) {
        final boolean nearBlue =
                pos.getDistance(FieldConstants.REEF_CENTER_BLUE) < pos.getDistance(FieldConstants.REEF_CENTER_RED);
        final double[] reefX = nearBlue ? BLUE_REEF_X : RED_REEF_X;
        final double[] reefY = nearBlue ? BLUE_REEF_Y : RED_REEF_Y;

        final double x = pos.getX();
        final double y = pos.getY();

        double nearestNormSquared = Double.POSITIVE_INFINITY;
        double nearestX = 0, nearestY = 0;
        double nearestEdgeVecX = 0, nearestEdgeVecY = 0;

        double pointAx, pointAy;
        double pointBx = reefX[5];
        double pointBy = reefY[5];
        for (int i = 0; i < 6; i++) { // loop over edges
            pointAx = pointBx;
            pointAy = pointBy;

            pointBx = reefX[i];
            pointBy = reefY[i];

            double t = (x - pointAx) * REEF_EDGE_VECTORS_X[i]
                    + (y - pointAy) * REEF_EDGE_VECTORS_Y[i]; // dot product (pos - A) * edge vector
            t = MathUtil.clamp(t, 0, 1);

            // closest point on this edge
            double possNearestX = (1 - t) * pointAx + t * pointBx;
            double possNearestY = (1 - t) * pointAy + t * pointBy;

            double possNormSquared = (x - possNearestX) * (x - possNearestX) + (y - possNearestY) * (y - possNearestY);

            if (possNormSquared < nearestNormSquared) {
                nearestNormSquared = possNormSquared;
                nearestX = possNearestX;
                nearestY = possNearestY;
                nearestEdgeVecX = REEF_EDGE_VECTORS_X[i];
                nearestEdgeVecY = REEF_EDGE_VECTORS_Y[i];
            }
        }

        // form pose from point and rotation from rotated edge vector
        return new Pose2d(nearestX, nearestY, new Rotation2d(nearestEdgeVecY, -nearestEdgeVecX));
    }

    public static final Pose2d nearestBoundaryPose(Translation2d pos) {
        Pose2d nearestPerimeter = nearestPerimeterPose(pos);
        Pose2d nearestReef = nearestReefPose(pos);
        if (nearestPerimeter.getTranslation().getDistance(pos)
                < nearestReef.getTranslation().getDistance(pos)) {
            return nearestPerimeter;
        } else {
            return nearestReef;
        }
    }

    // adjust velocity to not go into walls when facing them
    public static final Translation2d adjustVelocity(Pose2d pose, Translation2d desiredVel) {
        Pose2d nearestPoint = nearestBoundaryPose(pose.getTranslation());

        Logger.recordOutput("Boundaries/Closest Boundary Point", nearestPoint);

        Translation2d wallNormalInwards = new Translation2d(
                        Math.cos(nearestPoint.getRotation().getRadians()),
                        Math.sin(nearestPoint.getRotation().getRadians()))
                .unaryMinus();

        double minDistanceToWall = IntakeConstants.DISTANCE_TO_KEEP_FROM_WALL.in(Meters)
                + offsetDistanceFromRotation(pose.getRotation(), wallNormalInwards.unaryMinus());

        // bot is not near wall, or not facing wall, return desired velocity
        if (nearestPoint.getTranslation().getDistance(pose.getTranslation()) > minDistanceToWall
                || Math.abs(pose.getRotation().minus(nearestPoint.getRotation()).getRadians())
                        > IntakeConstants.ANGLE_TO_FACE_WALL.in(Radians)) {
            return desiredVel;
        }

        double velTowardsWall = desiredVel.getX() * wallNormalInwards.getX()
                + desiredVel.getY() * wallNormalInwards.getY(); // dot product
        if (velTowardsWall <= 0) { // not going towards wall
            return desiredVel;
        }

        Translation2d adjustedVel = desiredVel.minus(wallNormalInwards.times(velTowardsWall));
        Logger.recordOutput("Boundaries/Desired Velocity", new Pose2d(pose.getTranslation(), desiredVel.getAngle()));
        Logger.recordOutput("Boundaries/Adjusted Velocity", new Pose2d(pose.getTranslation(), adjustedVel.getAngle()));
        return adjustedVel;
    }

    private static double offsetDistanceFromRotation(Rotation2d robotRotation, Translation2d wallNormal) {
        double robotApothem = Dimensions.ROBOT_SIZE.div(2).in(Meters);
        double extension = IntakeConstants.INTAKE_EXTENSION.in(Meters);
        double h = Math.hypot(extension + robotApothem, robotApothem); // distance from robot center to intake tip
        double intakeAngle = Math.atan2(robotApothem, extension + robotApothem); // angle from forward to intake tip
        double angleFromWall =
                Math.abs(wallNormal.getAngle().minus(robotRotation).getRadians()); // absolute angle from
        // facing wall

        return h * Math.cos(angleFromWall - intakeAngle);
    }

    public static boolean inBargeZone(Translation2d pos) {
        return pos.getX() > FieldConstants.BARGE_X.in(Meters)
                && pos.getX() < VisionConstants.APRIL_TAGS.getFieldLength() - FieldConstants.BARGE_X.in(Meters);
    }

    // prevent instantiation
    private BoundaryProtections() {}
}
