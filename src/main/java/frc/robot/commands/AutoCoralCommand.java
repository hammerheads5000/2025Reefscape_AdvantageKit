// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.AlignConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.EndEffectorConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.coraldetection.CoralDetection;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.endeffector.EndEffector;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.swerve.Swerve;
import java.util.Optional;
import java.util.Set;
import org.littletonrobotics.junction.Logger;

/**
 * Detects a coral, plans a single approach pose, drives there with {@link AlignToPoseCommand}, then backs the
 * rear-mounted intake into the piece while the intake runs.
 */
public class AutoCoralCommand extends SequentialCommandGroup {
    private static final double EPSILON = 1e-4;
    private static final double DEFAULT_APPROACH_RANGE = 1.8; // m, used if distance estimate fails
    private static final double FINAL_BACK_SPEED = 0.8; // m/s during final pickup
    private static final double FINAL_BACK_TIMEOUT = 1.0; // s limit for final pickup push
    private static final double REPLAN_DISTANCE_METERS = Units.feetToMeters(2.0);

    private final Swerve swerve;
    private final Intake intake;
    private final EndEffector endEffector;
    private final Elevator elevator;
    private final CoralDetection coralDetection;

    private double lastDistanceFeet = Double.NaN;
    private double approachStartTime = 0.0;
    private Translation2d approachDirection = null;
    private Translation2d currentCoralField = null;
    private Pose2d lastApproachPose = null;
    private boolean replanTriggered = false;

    public AutoCoralCommand(
            Swerve swerve, Intake intake, EndEffector endEffector, Elevator elevator, CoralDetection coralDetection) {
        this.swerve = swerve;
        this.intake = intake;
        this.endEffector = endEffector;
        this.coralDetection = coralDetection;
        this.elevator = elevator;

        addRequirements(swerve, intake, endEffector, elevator);

        addCommands(Commands.defer(this::buildSequence, Set.of(swerve, intake, endEffector, elevator)));
    }

    private Command buildSequence() {
        currentCoralField = null;
        lastApproachPose = null;
        replanTriggered = false;
        Logger.recordOutput("AutoCoral/ReplanTriggered", false);

        Optional<Pose2d> approachPoseOpt = calculateApproachPose();
        if (approachPoseOpt.isEmpty()) {
            return Commands.runOnce(() -> Logger.recordOutput("AutoCoral/ApproachPoseValid", false));
        }

        Pose2d approachPose = approachPoseOpt.get();
        Logger.recordOutput("AutoCoral/ApproachPose", approachPose);

        AlignToPoseCommand initialAlign = createAlignCommand(approachPose);
        Command replanMonitor = Commands.waitUntil(this::shouldReplan);
        Command approachPhase = Commands.sequence(
                Commands.race(initialAlign, replanMonitor),
                Commands.defer(this::buildReplanAlign, Set.of(swerve, intake, endEffector, elevator)));

        Command alignWithIntake = Commands.sequence(Commands.runOnce(() -> startIntake(approachPose)), approachPhase);

        Command finalBack = Commands.deadline(
                        Commands.waitUntil(intake.coralDetectedTrigger).withTimeout(FINAL_BACK_TIMEOUT),
                        Commands.run(
                                () -> {
                                    Translation2d driveVector;
                                    if (approachDirection != null && approachDirection.getNorm() > EPSILON) {
                                        driveVector = approachDirection.times(FINAL_BACK_SPEED);
                                    } else {
                                        driveVector =
                                                new Translation2d(-FINAL_BACK_SPEED, 0).rotateBy(swerve.getRotation());
                                    }
                                    swerve.driveFieldCentric(
                                            MetersPerSecond.of(driveVector.getX()),
                                            MetersPerSecond.of(driveVector.getY()),
                                            RadiansPerSecond.zero());
                                },
                                swerve))
                .finallyDo(() -> swerve.stop());

        return Commands.sequence(alignWithIntake, finalBack).finallyDo(() -> {
            stopIntake();
            if (!Double.isNaN(lastDistanceFeet)) {
                System.out.printf("Coral %.2f ft away%n", lastDistanceFeet);
            }
        });
    }

    private Optional<Pose2d> calculateApproachPose() {
        approachDirection = null;

        var measurement = coralDetection.getClosestCoral();
        if (measurement == null) {
            return Optional.empty();
        }

        Optional<Translation2d> coralFieldOpt = estimateCoralOnField(measurement);
        Translation2d robotTranslation = swerve.getPose().getTranslation();
        Translation2d coralField;
        double distanceMeters;

        if (coralFieldOpt.isPresent()) {
            coralField = coralFieldOpt.get();
            distanceMeters = coralField.minus(robotTranslation).getNorm();
        } else {
            // Fallback: project along camera yaw using default range. Include camera mount yaw so we
            // respect its rear-facing orientation when estimating the ray direction.
            double yawRadians = Math.toRadians(measurement.getX());
            Rotation2d cameraYaw = Rotation2d.fromRadians(yawRadians);
            Rotation2d cameraMountYaw =
                    VisionConstants.CORAL_CAM_POS.getRotation().toRotation2d();
            Rotation2d fieldHeading = swerve.getRotation().plus(cameraMountYaw).plus(cameraYaw);
            Translation2d guess = new Translation2d(DEFAULT_APPROACH_RANGE, 0).rotateBy(fieldHeading);
            coralField = robotTranslation.plus(guess);
            distanceMeters = DEFAULT_APPROACH_RANGE;
        }

        currentCoralField = coralField;
        lastDistanceFeet = Units.metersToFeet(distanceMeters);
        Logger.recordOutput("AutoCoral/CoralDistanceFt", lastDistanceFeet);

        Translation2d toCoral = coralField.minus(robotTranslation);
        if (toCoral.getNorm() < EPSILON) {
            return Optional.empty();
        }

        Translation2d directionToCoral = toCoral.div(toCoral.getNorm());
        Translation2d approachTranslation =
                coralField.minus(directionToCoral.times(IntakeConstants.CORAL_APPROACH_OFFSET.in(Meters)));
        approachDirection = directionToCoral;
        Rotation2d approachRotation = directionToCoral.getAngle().plus(Rotation2d.kPi); // back (intake) faces coral
        Pose2d approach = new Pose2d(approachTranslation, approachRotation);
        lastApproachPose = approach;

        return Optional.of(approach);
    }

    private Optional<Translation2d> estimateCoralOnField(Translation2d measurement) {
        double yawRadians = Math.toRadians(measurement.getX());
        double pitchRadians = Math.toRadians(measurement.getY());

        Translation3d directionCamera = new Translation3d(
                Math.cos(pitchRadians) * Math.cos(yawRadians),
                Math.cos(pitchRadians) * Math.sin(yawRadians),
                Math.sin(pitchRadians));

        Pose3d cameraPose = new Pose3d(swerve.getPose()).transformBy(VisionConstants.CORAL_CAM_POS);
        Translation3d cameraPosition = cameraPose.getTranslation();
        Rotation3d cameraRotation = cameraPose.getRotation();

        Translation3d directionField = directionCamera.rotateBy(cameraRotation);
        double dz = directionField.getZ();
        if (Math.abs(dz) < EPSILON) {
            return Optional.empty();
        }

        double t = -cameraPosition.getZ() / dz;
        if (t <= 0) {
            return Optional.empty();
        }

        Translation3d coralPosition = cameraPosition.plus(directionField.times(t));
        return Optional.of(new Translation2d(coralPosition.getX(), coralPosition.getY()));
    }

    private void startIntake(Pose2d approachPose) {
        intake.setIntakeSpeed(IntakeConstants.INTAKE_SPEED);
        intake.setAlignSpeed(IntakeConstants.ALIGN_SPEED);
        endEffector.setSpeed(EndEffectorConstants.INTAKE_SPEED);
        elevator.setGoal(ElevatorConstants.INTAKE_HEIGHT);
        approachStartTime = Timer.getFPGATimestamp();
        Logger.recordOutput("AutoCoral/ApproachTarget", approachPose);
    }

    private void stopIntake() {
        intake.setIntakeSpeed(Volts.zero());
        intake.setAlignSpeed(Volts.zero());
        endEffector.setSpeed(Volts.zero());
    }

    private boolean shouldReplan() {
        if (currentCoralField == null) {
            return false;
        }
        Translation2d robotTranslation = swerve.getPose().getTranslation();
        return currentCoralField.minus(robotTranslation).getNorm() <= REPLAN_DISTANCE_METERS;
    }

    private AlignToPoseCommand createAlignCommand(Pose2d pose) {
        return new AlignToPoseCommand(
                pose, AlignConstants.CORAL_PICKUP_PID_TRANSLATION, AlignConstants.CORAL_PICKUP_PID_ANGLE, swerve);
    }

    private Command buildReplanAlign() {
        Optional<Pose2d> replanPoseOpt = calculateApproachPose();
        if (replanPoseOpt.isPresent()) {
            replanTriggered = true;
            Logger.recordOutput("AutoCoral/ReplanTriggered", true);
            Logger.recordOutput("AutoCoral/ApproachPose", replanPoseOpt.get());
            return createAlignCommand(replanPoseOpt.get());
        }

        if (lastApproachPose != null) {
            Logger.recordOutput("AutoCoral/ReplanTriggered", true);
            return createAlignCommand(lastApproachPose);
        }

        return Commands.none();
    }
}
