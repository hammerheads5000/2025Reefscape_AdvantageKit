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

    private final Swerve swerve;
    private final Intake intake;
    private final EndEffector endEffector;
    private final CoralDetection coralDetection;

    private double lastDistanceFeet = Double.NaN;
    private double approachStartTime = 0.0;

    public AutoCoralCommand(
            Swerve swerve, Intake intake, EndEffector endEffector, Elevator elevator, CoralDetection coralDetection) {
        this.swerve = swerve;
        this.intake = intake;
        this.endEffector = endEffector;
        this.coralDetection = coralDetection;

        addRequirements(swerve, intake, endEffector);

        addCommands(Commands.defer(this::buildSequence, Set.of(swerve, intake, endEffector)));
    }

    private Command buildSequence() {
        Optional<Pose2d> approachPoseOpt = calculateApproachPose();
        if (approachPoseOpt.isEmpty()) {
            return Commands.runOnce(() -> Logger.recordOutput("AutoCoral/ApproachPoseValid", false));
        }

        Pose2d approachPose = approachPoseOpt.get();
        Logger.recordOutput("AutoCoral/ApproachPose", approachPose);

        AlignToPoseCommand align = new AlignToPoseCommand(
                approachPose,
                AlignConstants.CORAL_PICKUP_PID_TRANSLATION,
                AlignConstants.CORAL_PICKUP_PID_ANGLE,
                swerve);

        Command alignWithIntake = Commands.sequence(Commands.runOnce(() -> startIntake(approachPose)), align);

        Command finalBack = Commands.deadline(
                        Commands.waitUntil(intake.coralDetectedTrigger).withTimeout(FINAL_BACK_TIMEOUT),
                        Commands.run(
                                () -> {
                                    Translation2d backwards =
                                            new Translation2d(-FINAL_BACK_SPEED, 0).rotateBy(swerve.getRotation());
                                    swerve.driveFieldCentric(
                                            MetersPerSecond.of(backwards.getX()),
                                            MetersPerSecond.of(backwards.getY()),
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
            // Fallback: project along camera yaw using default range
            double yawRadians = Math.toRadians(measurement.getX());
            Rotation2d cameraYaw = Rotation2d.fromRadians(yawRadians);
            Rotation2d fieldHeading = swerve.getRotation().plus(cameraYaw);
            Translation2d guess = new Translation2d(DEFAULT_APPROACH_RANGE, 0).rotateBy(fieldHeading);
            coralField = robotTranslation.plus(guess);
            distanceMeters = DEFAULT_APPROACH_RANGE;
        }

        lastDistanceFeet = Units.metersToFeet(distanceMeters);
        Logger.recordOutput("AutoCoral/CoralDistanceFt", lastDistanceFeet);

        Translation2d toCoral = coralField.minus(robotTranslation);
        if (toCoral.getNorm() < EPSILON) {
            return Optional.empty();
        }

        Translation2d directionToCoral = toCoral.div(toCoral.getNorm());
        Translation2d approachTranslation =
                coralField.plus(directionToCoral.times(IntakeConstants.CORAL_APPROACH_OFFSET.in(Meters)));
        Rotation2d approachRotation =
                directionToCoral.getAngle().plus(Rotation2d.kPi); // rear faces coral

        return Optional.of(new Pose2d(approachTranslation, approachRotation));
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
        approachStartTime = Timer.getFPGATimestamp();
        Logger.recordOutput("AutoCoral/ApproachTarget", approachPose);
    }

    private void stopIntake() {
        intake.setIntakeSpeed(Volts.zero());
        intake.setAlignSpeed(Volts.zero());
        endEffector.setSpeed(Volts.zero());
    }
}
