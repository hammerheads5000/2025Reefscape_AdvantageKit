// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.coraldetection;

import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.swerve.Swerve;

import java.util.List;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

public class CoralDetection extends SubsystemBase {
    private CoralDetectionIO io;
    private CoralDetectionIOInputsAutoLogged inputs;
    private Supplier<Pose2d> poseSupplier;

    private final Matrix<N3,N3> CAMERA_ROTATION_MAT;

    private Alert disconnectedAlert;

    private List<Translation2d> coralList = List.of();

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
    }

    private Matrix<N3,N3> calculateRotationMat() {
        double roll = VisionConstants.CORAL_CAM_POS.getRotation().getX();
        double pitch = VisionConstants.CORAL_CAM_POS.getRotation().getY();
        double yaw = VisionConstants.CORAL_CAM_POS.getRotation().getZ();

        return MatBuilder.fill(Nat.N3(), Nat.N3(), 
            Math.cos(pitch)*Math.cos(roll), Math.sin(yaw)*Math.sin(pitch)*Math.cos(roll)-Math.cos(yaw)*Math.sin(roll), Math.cos(yaw)*Math.sin(pitch)*Math.cos(roll)+Math.sin(yaw)*Math.sin(roll),
            Math.cos(pitch)*Math.sin(roll), Math.sin(yaw)*Math.sin(pitch)*Math.sin(roll)+Math.cos(yaw)*Math.cos(roll), Math.cos(yaw)*Math.sin(pitch)*Math.sin(roll)-Math.sin(yaw)*Math.cos(roll),
            -Math.sin(pitch), Math.sin(yaw)*Math.cos(pitch), Math.cos(yaw)*Math.cos(pitch));
    }

    public Translation2d getClosestCoral() {
        // Translation2d closest = null;
        // for (var coral : inputs.corals) {
        //     if (closest == null || coral.getY() < closest.getY()) {
        //         closest = coral;
        //     }
        // }
        // return closest;
        Translation2d closest = null;
        Translation2d robotPos = poseSupplier.get().getTranslation();
        for (var coral : coralList) {
            if (closest == null || coral.getDistance(robotPos) < closest.getDistance(robotPos)) {
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
        coralList = List.of(corals).stream()
                .map(this::projectCoralPosition)
                .map(this::robotToFieldRelative)
                .toList();
    }

    // transforms a coral position in (yaw, pitch) to a 2D position in robot space (x, y)
    private Translation2d projectCoralPosition(Translation2d coral) {
        double yaw = Math.toRadians(coral.getX());
        double pitch = Math.toRadians(coral.getY());

        // 3D extrinsic rotation matrix (roll, pitch, yaw order)
        Matrix<N3, N1> rayDirectionFromCam = MatBuilder.fill(Nat.N3(), Nat.N1(),
                Math.cos(pitch) * Math.sin(yaw),
                Math.cos(pitch) * Math.cos(yaw),
                Math.sin(pitch));

        // Robot oriented
        Matrix<N3, N1> rayDirectionFromBot = CAMERA_ROTATION_MAT.times(rayDirectionFromCam);
        
        Translation3d camPos = VisionConstants.CORAL_CAM_POS.getTranslation();

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
}
