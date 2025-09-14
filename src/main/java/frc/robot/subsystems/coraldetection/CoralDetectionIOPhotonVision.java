// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.coraldetection;

import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.subsystems.coraldetection.CoralDetectionIO.CoralDetectionIOInputs;
import java.util.LinkedList;
import java.util.List;
import org.littletonrobotics.junction.Logger;
import org.photonvision.PhotonCamera;

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

        List<Translation2d> observations = new LinkedList<>();
        for (var result : camera.getAllUnreadResults()) {
            if (result.hasTargets()) {
                observations.add(new Translation2d(
                        result.getBestTarget().getYaw(), result.getBestTarget().getPitch()));
            }
        }

        inputs.corals = observations.toArray(Translation2d[]::new);
        Logger.recordOutput("CoralDetection/Corals", inputs.corals);
    }
}
