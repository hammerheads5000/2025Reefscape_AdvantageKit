// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.coraldetection;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
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

public class CoralDetection extends SubsystemBase {
    private CoralDetectionIO io;
    private CoralDetectionIOInputsAutoLogged inputs;
    private Supplier<Pose2d> poseSupplier;

    private final Matrix<N3, N3> CAMERA_ROTATION_MAT;

    private Alert disconnectedAlert;

    private List<Translation2d> coralList = List.of();

    @AutoLogOutput
    public Trigger hasTarget = new Trigger(() -> inputs.corals.length > 0);

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

    private Matrix<N3, N3> calculateRotationMat() {
        double roll = VisionConstants.CORAL_CAM_POS.getRotation().getX();
        double pitch = VisionConstants.CORAL_CAM_POS.getRotation().getY();
        double yaw = VisionConstants.CORAL_CAM_POS.getRotation().getZ();

        return MatBuilder.fill(
                Nat.N3(),
                Nat.N3(),
                Math.cos(pitch) * Math.cos(roll),
                Math.sin(yaw) * Math.sin(pitch) * Math.cos(roll) - Math.cos(yaw) * Math.sin(roll),
                Math.cos(yaw) * Math.sin(pitch) * Math.cos(roll) + Math.sin(yaw) * Math.sin(roll),
                Math.cos(pitch) * Math.sin(roll),
                Math.sin(yaw) * Math.sin(pitch) * Math.sin(roll) + Math.cos(yaw) * Math.cos(roll),
                Math.cos(yaw) * Math.sin(pitch) * Math.sin(roll) - Math.sin(yaw) * Math.cos(roll),
                -Math.sin(pitch),
                Math.sin(yaw) * Math.cos(pitch),
                Math.cos(yaw) * Math.cos(pitch));
    }

    private void logCorals() {
        Pose3d[] poses = coralList.stream()
                .map(translation -> new Pose3d(new Translation3d(translation), new Rotation3d()))
                .toArray(Pose3d[]::new);
        Logger.recordOutput("CoralDetection/CoralPoses", poses);
    }

    @AutoLogOutput
    public Translation2d getClosestCoral() {
        return getClosestCoral(false);
    }

    public Translation2d getClosestCoral(boolean ignoreWall) {
        Translation2d closest = null;
        Translation2d robotPos = poseSupplier.get().getTranslation();
        for (var coral : coralList) {
            if (closest == null
                    || coral.getDistance(robotPos) < closest.getDistance(robotPos)
                            && BoundaryProtections.nearestBoundaryPose(coral)
                                            .getTranslation()
                                            .getDistance(coral)
                                    <= IntakeConstants.CORAL_ON_WALL_THRESHOLD.in(Meters)) {
                closest = coral;
            }
        }
        return closest;
    }

    private void updateCoralList() {
        Translation2d[] corals = inputs.corals;
        if (corals.length == 0) {
            return;
        }
        if (Constants.CURRENT_MODE == Constants.SIM_MODE) {
            // In sim, the corals are already in field-relative coordinates
            coralList = List.of(corals);
            return;
        }
        coralList = List.of(corals).stream()
                .map(this::projectCoralPosition)
                .map(this::robotToFieldRelative)
                // .filter(this::coralInBounds)
                .toList();
    }

    // transforms a coral position in (yaw, pitch) to a 2D position in robot space (x, y)
    private Translation2d projectCoralPosition(Translation2d coral) {
        double yaw = Math.toRadians(coral.getX());
        double pitch = Math.toRadians(coral.getY());

        Vector<N3> rayDirectionFromCam =
                VecBuilder.fill(Math.cos(pitch) * Math.cos(yaw), Math.cos(pitch) * Math.sin(yaw), Math.sin(pitch));

        Translation3d camPos = VisionConstants.CORAL_CAM_POS.getTranslation();

        // Robot oriented
        Matrix<N3, N1> rayDirectionFromBot = CAMERA_ROTATION_MAT.times(rayDirectionFromCam);

        // Line: camPos + t*rayDirection = [x, y, 0]
        double t = -camPos.getZ() / rayDirectionFromBot.get(2, 0);
        double x = camPos.getX() + t * rayDirectionFromBot.get(0, 0);
        double y = camPos.getY() + t * rayDirectionFromBot.get(1, 0);

        return new Translation2d(x, y);
    }

    private Translation2d robotToFieldRelative(Translation2d pos) {
        Pose2d robotPose = poseSupplier.get();
        return robotPose.getTranslation().plus(pos.rotateBy(robotPose.getRotation()));
    }

    private boolean coralInBounds(Translation2d pos) {
        return pos.getX() >= 0
                && pos.getX() <= VisionConstants.APRIL_TAGS.getFieldLength()
                && pos.getY() >= 0
                && pos.getY() <= VisionConstants.APRIL_TAGS.getFieldWidth();
    }
}
