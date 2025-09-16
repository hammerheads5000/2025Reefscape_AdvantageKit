// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.AlignConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.EndEffectorConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.NTConstants;
import frc.robot.Constants.PathConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.coraldetection.CoralDetection;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.endeffector.EndEffector;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.swerve.Swerve;
import java.util.Optional;
import java.util.Set;
import java.util.function.Supplier;
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
    private static final double REPLAN_DISTANCE_METERS = Units.feetToMeters(3.0);
    private static final double ORIENTATION_HOLD_DEGREES = 8.0;

    private final Swerve swerve;
    private final Intake intake;
    private final EndEffector endEffector;
    private final Elevator elevator;
    private final CoralDetection coralDetection;

    private double lastDistanceFeet = Double.NaN;
    private double approachStartTime = 0.0;
    private Translation2d approachDirection = null;
    private Translation2d currentCoralField = null;
    private Pose2d currentApproachPose = null;
    private boolean replanLogged = false;
    private boolean scoreScheduled = false;

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
        currentApproachPose = null;
        replanLogged = false;
        Logger.recordOutput("AutoCoral/ReplanTriggered", false);

        Optional<Pose2d> approachPoseOpt = calculateApproachPose();
        if (approachPoseOpt.isEmpty()) {
            return Commands.runOnce(() -> Logger.recordOutput("AutoCoral/ApproachPoseValid", false));
        }

        Pose2d approachPose = approachPoseOpt.get();
        Logger.recordOutput("AutoCoral/ApproachPose", approachPose);

        AlignToPoseCommand alignCommand = createAlignCommand(() -> currentApproachPose);
        Command replanWatcher = Commands.run(() -> {
            if (shouldReplan()) {
                Optional<Pose2d> newPose = calculateApproachPose();
                if (newPose.isPresent()) {
                    Logger.recordOutput("AutoCoral/ApproachPose", newPose.get());
                    if (!replanLogged) {
                        replanLogged = true;
                        Logger.recordOutput("AutoCoral/ReplanTriggered", true);
                    }
                }
            }
        });

        Command approachPhase = Commands.deadline(alignCommand, replanWatcher);

        Command alignWithIntake = Commands.sequence(
                Commands.runOnce(() -> startIntake(approachPose)),
                Commands.deadline(Commands.run(this::updateSpinState), approachPhase));

        Command finalBack = Commands.deadline(
                        Commands.waitUntil(() -> intake.coralDetectedTrigger.getAsBoolean())
                                .withTimeout(FINAL_BACK_TIMEOUT),
                        Commands.run(
                                () -> {
                                    updateSpinState();
                                    boolean hasCoral = endEffector.hasCoral();
                                    boolean lidarTriggered = intake.alignerHasPiece();
                                    boolean currentSpike = intake.coralDetectedTrigger.getAsBoolean();

                                    if (!scoreScheduled && (currentSpike || hasCoral)) {
                                        Command scoreCommand = createScoreSequence();
                                        if (scoreCommand != null) {
                                            CommandScheduler.getInstance().schedule(scoreCommand);
                                            scoreScheduled = true;
                                        }
                                    }

                                    boolean continueDriving = !hasCoral && !lidarTriggered && !currentSpike;

                                    if (continueDriving
                                            && approachDirection != null
                                            && approachDirection.getNorm() > EPSILON) {
                                        Translation2d driveVector = approachDirection.times(FINAL_BACK_SPEED);
                                        swerve.driveFieldCentric(
                                                MetersPerSecond.of(driveVector.getX()),
                                                MetersPerSecond.of(driveVector.getY()),
                                                RadiansPerSecond.zero());
                                    } else {
                                        swerve.stop();
                                    }

                                    Logger.recordOutput("AutoCoral/IntakeCurrentDetected", currentSpike);
                                    Logger.recordOutput("AutoCoral/ContinueDriving", continueDriving);
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
        distanceMeters = coralField.minus(robotTranslation).getNorm();
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
        Pose2d approach =
                new Pose2d(approachTranslation, directionToCoral.getAngle().plus(Rotation2d.kPi));
        currentApproachPose = approach;

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
        intake.setGuardEnabled(false);
        stopIntakeMotors();
        elevator.setGoal(ElevatorConstants.INTAKE_HEIGHT);
        approachStartTime = Timer.getFPGATimestamp();
        targetActive = false;
        scoreScheduled = false;
        Logger.recordOutput("AutoCoral/ApproachTarget", approachPose);
    }

    private void stopIntake() {
        intake.setGuardEnabled(true);
        stopIntakeMotors();
    }

    private boolean targetActive = false;

    private void updateSpinState() {
        boolean hasCoral = endEffector.hasCoral();
        boolean withinRange = currentCoralField != null
                && currentCoralField.minus(swerve.getPose().getTranslation()).getNorm() <= Units.feetToMeters(6.0);
        if (!targetActive && withinRange && !hasCoral) {
            intake.setIntakeSpeed(IntakeConstants.INTAKE_SPEED);
            intake.setAlignSpeed(IntakeConstants.ALIGN_SPEED);
            endEffector.setSpeed(EndEffectorConstants.INTAKE_SPEED);
            targetActive = true;
        } else if (targetActive && hasCoral) {
            stopIntakeMotors();
            targetActive = false;
        } else if (!targetActive && hasCoral) {
            stopIntakeMotors();
        }
    }

    private void stopIntakeMotors() {
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

    private AlignToPoseCommand createAlignCommand(Supplier<Pose2d> poseSupplier) {
        return new AlignToPoseCommand(
                poseSupplier,
                AlignConstants.CORAL_PICKUP_PID_TRANSLATION,
                AlignConstants.CORAL_PICKUP_PID_ANGLE,
                swerve,
                ORIENTATION_HOLD_DEGREES);
    }

    private Command createScoreSequence() {
        String descriptor = NTConstants.REEF_TELEOP_AUTO_ENTRY.get();
        if (descriptor == null || descriptor.length() < 2) {
            return null;
        }

        char branch = descriptor.charAt(0);
        Pair<Integer, Integer> sidePosPair = FieldConstants.LETTER_TO_SIDE_AND_RELATIVE.get(Character.valueOf(branch));
        if (sidePosPair == null) {
            return null;
        }

        int side = sidePosPair.getFirst().intValue();
        int relative = sidePosPair.getSecond().intValue();
        char level = descriptor.charAt(1);

        double relativePos = relative;
        Command endEffectorCommand;
        if (level == '1') {
            endEffectorCommand = relative > 0 ? endEffector.troughLeftCommand() : endEffector.troughRightCommand();
            relativePos *= PathConstants.L1_RELATIVE_POS;
        } else {
            endEffectorCommand = endEffector.scoreCommand();
        }

        ApproachReefCommand approach = new ApproachReefCommand(side, relativePos, swerve);
        Command elevatorCommand = getElevatorTrackCommand(level, approach::getDistanceToTarget);

        return Commands.sequence(
                Commands.parallel(approach, elevatorCommand),
                Commands.waitTime(PathConstants.AFTER_WAIT_TIME),
                endEffectorCommand.asProxy(),
                Commands.waitTime(PathConstants.AFTER_WAIT_TIME),
                elevator.goToIntakePosCommand(false));
    }

    private Command getElevatorTrackCommand(char level, Supplier<Distance> distanceSupplier) {
        return switch (level) {
            case '1' -> elevator.trackL1Command(distanceSupplier);
            case '2' -> elevator.trackL2Command(distanceSupplier);
            case '3' -> elevator.trackL3Command(distanceSupplier);
            case '4' -> elevator.trackL4Command(distanceSupplier);
            default -> elevator.trackL4Command(distanceSupplier);
        };
    }
}
