// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import static edu.wpi.first.units.Units.Meters;
import static frc.robot.Constants.VisionConstants.ANGULAR_STD_DEV_BASELINE;
import static frc.robot.Constants.VisionConstants.APRIL_TAGS;
import static frc.robot.Constants.VisionConstants.LINEAR_STD_DEV_BASELINE;
import static frc.robot.Constants.VisionConstants.MAX_AMBIGUITY;
import static frc.robot.Constants.VisionConstants.MAX_Z_ERROR;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PathConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.vision.VisionIO.PoseObservation;
import java.util.LinkedList;
import java.util.List;
import java.util.Optional;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class Vision extends SubsystemBase {
    private final VisionConsumer consumer;
    private final VisionIO[] io;
    private final VisionIOInputsAutoLogged[] inputs;
    private final Alert[] disconnectedAlerts;
    private final ReefVisionIO reefIO;
    private final ReefVisionIOInputsAutoLogged reefInputs;
    private final Supplier<Pose2d> poseSupplier;

    public Vision(VisionConsumer consumer, Supplier<Pose2d> poseSupplier, ReefVisionIO reefIO, VisionIO... io) {
        this.consumer = consumer;
        this.io = io;
        this.reefIO = reefIO;
        this.poseSupplier = poseSupplier;

        this.inputs = new VisionIOInputsAutoLogged[io.length];
        for (int i = 0; i < inputs.length; i++) {
            inputs[i] = new VisionIOInputsAutoLogged();
        }

        this.disconnectedAlerts = new Alert[io.length];
        for (int i = 0; i < inputs.length; i++) {
            disconnectedAlerts[i] = new Alert("Camera " + i + " disconnected", AlertType.kWarning);
        }

        this.reefInputs = new ReefVisionIOInputsAutoLogged();
    }

    /**
     * Returns the X angle to the best target
     *
     * @param cameraIndex The index of the camera to use.
     */
    public Rotation2d getTargetX(int cameraIndex) {
        return inputs[cameraIndex].latestTarget.tx();
    }

    /**
     * Returns branch transform relative to robot via reef vision. Returns null if branch is farther than
     * PathConstants.SWITCH_TO_REEFVISION_DISTANCE
     */
    public Optional<Translation2d> getRelativeBranchPos() {
        if (reefInputs.distance.gt(PathConstants.SWITCH_TO_REEFVISION_DISTANCE)) {
            return Optional.empty();
        }
        return Optional.of(new Translation2d(reefInputs.distance, Meters.zero())
                .plus(VisionConstants.TOF_CAM_POS)
                .rotateBy(new Rotation2d(reefInputs.angle.unaryMinus())));
    }

    /**
     * Returns branch position on the field via reef vision. Returns empty if branch is farther than
     * PathConstants.SWITCH_TO_REEFVISION_DISTANCE
     */
    public Optional<Pose2d> getBranchPos() {
        return getRelativeBranchPos().map(pos -> poseSupplier.get().plus(new Transform2d(pos, Rotation2d.kZero)));
    }

    @Override
    public void periodic() {
        for (int i = 0; i < io.length; i++) {
            io[i].updateInputs(inputs[i]);
            Logger.processInputs("Vision/Camera" + i, inputs[i]);
        }

        List<Pose3d> allTagPoses = new LinkedList<>();
        List<Pose3d> allRobotPoses = new LinkedList<>();
        List<Pose3d> allRobotPosesAccepted = new LinkedList<>();
        List<Pose3d> allRobotPosesRejected = new LinkedList<>();

        // Loop over cameras
        for (int cameraIdx = 0; cameraIdx < io.length; cameraIdx++) {
            // Get camera result
            CameraResult cameraResult = getCameraResult(cameraIdx);

            logCameraResult(cameraResult);

            // Add camera result to all results
            allTagPoses.addAll(cameraResult.tagPoses);
            allRobotPoses.addAll(cameraResult.robotPoses);
            allRobotPosesAccepted.addAll(cameraResult.robotPosesAccepted);
            allRobotPosesRejected.addAll(cameraResult.robotPosesRejected);
        }

        reefIO.updateInputs(reefInputs);
        Logger.processInputs("ReefVision", reefInputs);

        // Log summary data
        Logger.recordOutput("Vision/Summary/TagPoses", allTagPoses.toArray(new Pose3d[allTagPoses.size()]));
        Logger.recordOutput("Vision/Summary/RobotPoses", allRobotPoses.toArray(new Pose3d[allRobotPoses.size()]));
        Logger.recordOutput(
                "Vision/Summary/RobotPosesAccepted",
                allRobotPosesAccepted.toArray(new Pose3d[allRobotPosesAccepted.size()]));
        Logger.recordOutput(
                "Vision/Summary/RobotPosesRejected",
                allRobotPosesRejected.toArray(new Pose3d[allRobotPosesRejected.size()]));

        getBranchPos().ifPresent(pos -> Logger.recordOutput("ReefVision/BranchPos", pos));
    }

    /**
     * Returns the camera result for the given camera index.
     *
     * @param cameraIdx The index of the camera to get the result from.
     * @return The camera result.
     */
    private CameraResult getCameraResult(int cameraIdx) {
        // Update disconnected alert
        disconnectedAlerts[cameraIdx].set(!inputs[cameraIdx].connected);

        // Initialize logging values
        List<Pose3d> tagPoses = new LinkedList<>();
        List<Pose3d> robotPoses = new LinkedList<>();
        List<Pose3d> robotPosesAccepted = new LinkedList<>();
        List<Pose3d> robotPosesRejected = new LinkedList<>();

        // Add tag poses
        for (int tagId : inputs[cameraIdx].tagIds) {
            var tagPose = APRIL_TAGS.getTagPose(tagId);
            if (tagPose.isPresent()) {
                tagPoses.add(tagPose.get());
            }
        }

        for (PoseObservation observation : inputs[cameraIdx].poseObservations) {
            boolean rejectPose = !isObservationValid(observation);

            robotPoses.add(observation.pose());
            if (rejectPose) {
                robotPosesRejected.add(observation.pose());
            } else {
                robotPosesAccepted.add(observation.pose());
            }

            if (rejectPose) {
                continue;
            }

            double stdDevFactor = Math.pow(observation.averageTagDistance().in(Meters), 2.0) / observation.tagCount();
            double linearStdDev = LINEAR_STD_DEV_BASELINE * stdDevFactor;
            double angularStdDev = ANGULAR_STD_DEV_BASELINE * stdDevFactor;

            // Send vision observation
            consumer.accept(
                    observation.pose().toPose2d(),
                    observation.timestamp(),
                    VecBuilder.fill(linearStdDev, linearStdDev, angularStdDev));
        }

        return new CameraResult(cameraIdx, tagPoses, robotPoses, robotPosesAccepted, robotPosesRejected);
    }

    private void logCameraResult(CameraResult cameraResult) {
        String cameraTable = "Vision/Camera" + cameraResult.cameraIdx;
        Logger.recordOutput(
                cameraTable + "/TagPoses", cameraResult.tagPoses.toArray(new Pose3d[cameraResult.tagPoses.size()]));
        Logger.recordOutput(
                cameraTable + "/RobotPoses",
                cameraResult.robotPoses.toArray(new Pose3d[cameraResult.robotPoses.size()]));
        Logger.recordOutput(
                cameraTable + "/RobotPosesAccepted",
                cameraResult.robotPosesAccepted.toArray(new Pose3d[cameraResult.robotPosesAccepted.size()]));
        Logger.recordOutput(
                cameraTable + "/RobotPosesRejected",
                cameraResult.robotPosesRejected.toArray(new Pose3d[cameraResult.robotPosesRejected.size()]));
    }

    private boolean isObservationValid(PoseObservation observation) {
        boolean nonNull = observation.averageTagDistance() != null && observation.timestamp() != null;
        boolean hasTargets = observation.tagCount() > 0;
        boolean isAmbiguous = observation.tagCount() == 1 && observation.ambiguity() > MAX_AMBIGUITY;
        boolean unrealisticZ = Math.abs(observation.pose().getZ()) > MAX_Z_ERROR.in(Meters);
        boolean withinField = observation.pose().getX() >= 0.0
                && observation.pose().getY() >= 0.0
                && observation.pose().getX() <= APRIL_TAGS.getFieldLength()
                && observation.pose().getY() <= APRIL_TAGS.getFieldWidth();

        return nonNull && hasTargets && !isAmbiguous && !unrealisticZ && withinField;
    }

    @FunctionalInterface
    public static interface VisionConsumer {
        public void accept(Pose2d visionRobotPoseMeters, Time timestamp, Matrix<N3, N1> visionMeasurementStdDevs);
    }

    public static record CameraResult(
            int cameraIdx,
            List<Pose3d> tagPoses,
            List<Pose3d> robotPoses,
            List<Pose3d> robotPosesAccepted,
            List<Pose3d> robotPosesRejected) {}
}
