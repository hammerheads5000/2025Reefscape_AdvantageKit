// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.coraldetection;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.util.BoundaryProtections;
import java.util.List;
import java.util.function.Supplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

/** Manages coral seen by coral camera, and computes positions */
public class CoralDetection extends SubsystemBase {
    private CoralDetectionIO io;
    private CoralDetectionIOInputsAutoLogged inputs;
    private Supplier<Pose2d> poseSupplier;

    private final Matrix<N3, N3> CAMERA_ROTATION_MAT;

    private Alert disconnectedAlert;

    private List<Translation2d> coralList = List.of();

    @AutoLogOutput
    public Trigger hasTarget = new Trigger(() -> inputs.corals.length > 0).debounce(0.5, DebounceType.kFalling);

    /** Creates a new CoralDetection. */
    public CoralDetection(CoralDetectionIO io, Supplier<Pose2d> poseSupplier) {
        this.io = io;
        this.inputs = new CoralDetectionIOInputsAutoLogged();
        this.poseSupplier = poseSupplier;
        this.CAMERA_ROTATION_MAT = calculateRotationMat();

        this.disconnectedAlert = new Alert("Coral Camera disconnected", AlertType.kWarning);
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("CoralDetection", inputs);

        if (!inputs.connected) {
            disconnectedAlert.set(true);
            return;
        }
        disconnectedAlert.set(false);
        updateCoralList();
        logCorals();
    }

    /** Precalculate rotation matrix of camera to be stored */
    private Matrix<N3, N3> calculateRotationMat() {
        double roll = VisionConstants.CORAL_CAM_POS.getRotation().getX();
        double pitch = VisionConstants.CORAL_CAM_POS.getRotation().getY();
        double yaw = VisionConstants.CORAL_CAM_POS.getRotation().getZ();

        final Matrix<N3, N3> rollMat = MatBuilder.fill(
                Nat.N3(), Nat.N3(), 1, 0, 0, 0, Math.cos(roll), -Math.sin(roll), 0, Math.sin(roll), Math.cos(roll));

        final Matrix<N3, N3> pitchMat = MatBuilder.fill(
                Nat.N3(), Nat.N3(), Math.cos(pitch), 0, Math.sin(pitch), 0, 1, 0, -Math.sin(pitch), 0, Math.cos(pitch));

        final Matrix<N3, N3> yawMat = MatBuilder.fill(
                Nat.N3(), Nat.N3(), Math.cos(yaw), -Math.sin(yaw), 0, Math.sin(yaw), Math.cos(yaw), 0, 0, 0, 1);

        return yawMat.times(pitchMat).times(rollMat);
    }

    /** output corals to list of Pose3d's for visualization */
    private void logCorals() {
        Pose3d[] poses = coralList.stream()
                .map(translation -> new Pose3d(new Translation3d(translation), new Rotation3d()))
                .toArray(Pose3d[]::new);
        Logger.recordOutput("CoralDetection/CoralPoses", poses);
        Logger.recordOutput("CoralDetection/ClosestNon-WallCoral", getClosestCoral(true));
    }

    @AutoLogOutput
    public Translation2d getClosestCoral() {
        return getClosestCoral(false);
    }

    /** Returns closest coral to robot. Ignores coral close to walls if ignoreWall is true */
    public Translation2d getClosestCoral(boolean ignoreWall) {
        Translation2d closest = null;
        Translation2d robotPos = poseSupplier.get().getTranslation();
        for (var coral : coralList) {
            // if we care, make sure coral isn't on wall, and closest is null or farther than coral
            if ((!ignoreWall
                            || BoundaryProtections.nearestBoundaryPose(coral)
                                            .getTranslation()
                                            .getDistance(coral)
                                    > IntakeConstants.CORAL_ON_WALL_THRESHOLD.in(Meters))
                    && (closest == null || coral.getDistance(robotPos) < closest.getDistance(robotPos))) {
                closest = coral;
            }
        }
        return closest;
    }

    /** processes inputs to get list of coral positions on field */
    private void updateCoralList() {
        Translation2d[] corals = inputs.corals;
        if (corals.length == 0) {
            return;
        }
        if (Constants.CURRENT_MODE == Constants.SIM_MODE) {
            // In sim, the corals are already in field-relative coordinates
            coralList = List.of(corals).stream()
                    .filter(this::coralInRange)
                    .filter(this::coralInBounds)
                    .toList();
            return;
        }
        coralList = List.of(corals).stream()
                .map(this::projectCoralPosition)
                .map(this::robotToFieldRelative)
                .filter(this::coralInRange)
                .filter(this::coralInBounds)
                .toList();
    }

    /** transforms a coral position in (yaw, pitch) to a 2D position in robot space (x, y) */
    private Translation2d projectCoralPosition(Translation2d coral) {
        double yaw = -Math.toRadians(coral.getX());
        double pitch = Math.toRadians(coral.getY());

        Vector<N3> rayDirectionFromCam =
                VecBuilder.fill(Math.cos(pitch) * Math.cos(yaw), Math.cos(pitch) * Math.sin(yaw), Math.sin(pitch));

        Translation3d camPos = VisionConstants.CORAL_CAM_POS.getTranslation();

        // Robot oriented
        Matrix<N3, N1> rayDirectionFromBot = CAMERA_ROTATION_MAT.times(rayDirectionFromCam);

        // Line: camPos + t*rayDirection = [x, y, 2.25 in]
        double t = (Inches.of(2.25).in(Meters) - camPos.getZ()) / rayDirectionFromBot.get(2, 0);
        double x = camPos.getX() + t * rayDirectionFromBot.get(0, 0);
        double y = camPos.getY() + t * rayDirectionFromBot.get(1, 0);

        return new Translation2d(x, y);
    }

    /** transforms coral position in robot space to field space */
    private Translation2d robotToFieldRelative(Translation2d pos) {
        Pose2d robotPose = poseSupplier.get();
        return robotPose.getTranslation().plus(pos.rotateBy(robotPose.getRotation()));
    }

    private boolean coralInRange(Translation2d pos) {
        return pos.getDistance(poseSupplier.get().getTranslation()) <= IntakeConstants.MAX_CORAL_DISTANCE.in(Meters);
    }

    private boolean coralInBounds(Translation2d pos) {
        return (pos.getX() >= 0
                        && pos.getX() <= VisionConstants.APRIL_TAGS.getFieldLength()
                        && pos.getY() >= 0
                        && pos.getY() <= VisionConstants.APRIL_TAGS.getFieldWidth())
                && (pos.getX() <= VisionConstants.MAX_CORAL_X.in(Meters)
                        || pos.getX()
                                >= 2 * VisionConstants.APRIL_TAGS.getFieldLength()
                                        - VisionConstants.MAX_CORAL_X.in(Meters));
    }
}
