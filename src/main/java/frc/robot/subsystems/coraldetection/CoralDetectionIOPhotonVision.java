// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.coraldetection;

import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import frc.robot.subsystems.coraldetection.CoralDetectionIO.CoralDetectionIOInputs;
import java.util.Comparator;
import java.util.LinkedList;
import java.util.List;
import java.util.Optional;
import org.littletonrobotics.junction.Logger;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.TargetCorner;

/** Add your docs here. */
public class CoralDetectionIOPhotonVision implements CoralDetectionIO {
    protected final PhotonCamera camera;
    protected final Transform3d robotToCamera;

    /**
     * Creates a new VisionIOPhotonVision.
     *
     * @param name The configured name of the camera.
     * @param rotationSupplier The 3D position of the camera relative to the robot.
     */
    public CoralDetectionIOPhotonVision(String name, Transform3d robotToCamera) {
        camera = new PhotonCamera(name);
        this.robotToCamera = robotToCamera;
    }

    @Override
    public void updateInputs(CoralDetectionIOInputs inputs) {
        inputs.connected = camera.isConnected();

        Optional<Matrix<N3, N3>> cameraMatrixOpt = camera.getCameraMatrix();
        Optional<Matrix<N3, N3>> cameraMatrixInvOpt = cameraMatrixOpt.map(Matrix::inv);

        List<Translation2d> observations = new LinkedList<>();
        for (var result : camera.getAllUnreadResults()) {
            if (result.hasTargets()) {
                PhotonTrackedTarget target = result.getBestTarget();
                if (target == null) {
                    continue;
                }

                Translation2d measurement = cameraMatrixInvOpt
                        .flatMap(inv -> calculateBottomAngles(target, inv))
                        .orElseGet(() -> new Translation2d(target.getYaw(), target.getPitch()));
                observations.add(measurement);
            }
        }

        inputs.corals = observations.toArray(Translation2d[]::new);
        Logger.recordOutput("CoralDetection/Corals", inputs.corals);
    }

    private Optional<Translation2d> calculateBottomAngles(PhotonTrackedTarget target, Matrix<N3, N3> cameraMatrixInv) {
        List<TargetCorner> corners = target.getMinAreaRectCorners();
        if (corners == null || corners.size() < 2) {
            corners = target.getDetectedCorners();
        }
        if (corners == null || corners.size() < 2) {
            return Optional.empty();
        }

        List<TargetCorner> bottomCorners = corners.stream()
                .sorted(Comparator.comparingDouble((TargetCorner c) -> c.y).reversed())
                .limit(2)
                .toList();
        if (bottomCorners.size() < 2) {
            return Optional.empty();
        }

        double pixelX = bottomCorners.stream().mapToDouble(c -> c.x).average().orElse(Double.NaN);
        double pixelY = bottomCorners.stream().mapToDouble(c -> c.y).average().orElse(Double.NaN);
        if (!Double.isFinite(pixelX) || !Double.isFinite(pixelY)) {
            return Optional.empty();
        }

        Matrix<N3, N1> pixelVector = MatBuilder.fill(Nat.N3(), Nat.N1(), pixelX, pixelY, 1.0);
        Matrix<N3, N1> rayVector = cameraMatrixInv.times(pixelVector);

        double rx = rayVector.get(0, 0);
        double ry = rayVector.get(1, 0);
        double rz = rayVector.get(2, 0);

        double norm = Math.sqrt(rx * rx + ry * ry + rz * rz);
        if (norm < 1e-9) {
            return Optional.empty();
        }
        rx /= norm;
        ry /= norm;
        rz /= norm;

        double forward = rz;
        double left = -rx;
        double up = -ry;

        double horizontalMagnitude = Math.hypot(forward, left);
        double yawDeg;
        double pitchDeg;

        if (horizontalMagnitude < 1e-9) {
            yawDeg = 0.0;
            pitchDeg = Math.toDegrees(Math.copySign(Math.PI / 2.0, up));
        } else {
            yawDeg = Math.toDegrees(Math.atan2(left, forward));
            pitchDeg = Math.toDegrees(Math.atan2(up, horizontalMagnitude));
        }

        return Optional.of(new Translation2d(yawDeg, pitchDeg));
    }
}
