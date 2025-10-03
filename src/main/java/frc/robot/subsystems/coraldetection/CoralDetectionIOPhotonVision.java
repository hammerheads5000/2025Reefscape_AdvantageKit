// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.coraldetection;

import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import java.util.ArrayList;
import java.util.LinkedList;
import java.util.List;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

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
            observations.addAll(processResult(result));
        }

        inputs.corals = observations.toArray(Translation2d[]::new);
    }

    private List<Translation2d> processResult(PhotonPipelineResult result) {
        List<Translation2d> observations = new ArrayList<Translation2d>();
        if (!result.hasTargets()) {
            return observations;
        }

        for (PhotonTrackedTarget target : result.getTargets()) {
            if (target.getDetectedObjectClassID() == 1) {
                observations.add(new Translation2d(target.getYaw(), target.getPitch()));
            }
        }

        return observations;
    }
}
