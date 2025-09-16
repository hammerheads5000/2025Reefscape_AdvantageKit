// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.Dimensions;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.VisionConstants;
import java.util.function.Supplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {
    IntakeIO io;
    IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

    Trigger alignerHasPiece = new Trigger(() -> inputs.alignLidar).debounce(0.1);

    @AutoLogOutput
    Trigger deployedTrigger = new Trigger(this::isDeployed).debounce(0.1);

    @AutoLogOutput
    Trigger stowedTrigger = new Trigger(this::isStowed).debounce(0.1);

    @AutoLogOutput
    public Trigger coralDetectedTrigger = new Trigger(this::rawCoralDetected).debounce(0.3);

    private Angle goal = IntakeConstants.STOW_POS;

    IntakeVisualizer visualizer;
    private final Supplier<Pose2d> poseSupplier;
    private final double backExtensionMeters;
    private final double clearanceMeters;
    private boolean autoStowWarningIssued = false;
    private boolean autoStowed = false;
    private boolean stowSlowdownActive = false;
    private boolean guardEnabled = false;
    private double slowdownScale = 1.0;

    public Intake(IntakeIO io, Supplier<Pose2d> poseSupplier) {
        this.io = io;
        this.poseSupplier = poseSupplier;
        this.visualizer = new IntakeVisualizer(coralDetectedTrigger, poseSupplier);

        Distance halfRobot = frc.robot.Constants.Dimensions.ROBOT_SIZE.div(2);
        backExtensionMeters = halfRobot.plus(IntakeConstants.INTAKE_EXTENSION).in(Meters);
        clearanceMeters = IntakeConstants.DEPLOY_CLEARANCE.in(Meters);

        guardEnabled = true;

        SmartDashboard.putData("Intake Deploy", deployCommand(true));
        SmartDashboard.putData("Intake Stow", stowCommand(true));
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Intake", inputs);
        visualizer.update(inputs.position);

        if (guardEnabled) {
            boolean needsClearance = shouldAutoStowForClearance();

            if (isDeployed() && needsClearance) {
                if (!autoStowWarningIssued) {
                    DriverStation.reportWarning("Auto stowing intake to maintain wall clearance", false);
                    autoStowWarningIssued = true;
                }
                initiateStow();
                autoStowed = true;
            } else if (!isDeployed()) {
                autoStowWarningIssued = false;
                if (autoStowed && !needsClearance) {
                    setGoal(IntakeConstants.DEPLOY_POS);
                    autoStowed = false;
                }
            } else {
                autoStowWarningIssued = false;
                autoStowed = false;
            }

            if (stowSlowdownActive && inputs.position.in(Degrees) >= IntakeConstants.SLOWING_THRESHOLD.in(Degrees)) {
                io.setGoal(IntakeConstants.STOW_POS);
                stowSlowdownActive = false;
            }
        } else {
            autoStowWarningIssued = false;
            autoStowed = false;
            stowSlowdownActive = false;
        }

        slowdownScale = computeSpeedScale();
    }

    public void setIntakeSpeed(Voltage speed) {
        io.setIntakeSpeed(speed);
    }

    public void setAlignSpeed(Voltage speed) {
        io.setAlignSpeed(speed);
    }

    public void setGoal(Angle angle) {
        goal = angle;
        io.setGoal(angle);
    }

    public boolean alignerHasPiece() {
        return alignerHasPiece.getAsBoolean();
    }

    public Angle getPosition() {
        return inputs.position;
    }

    public boolean isDeployed() {
        return inputs.position.isNear(IntakeConstants.DEPLOY_POS, IntakeConstants.DEPLOY_TOLERANCE);
    }

    public boolean isStowed() {
        return inputs.position.isNear(IntakeConstants.STOW_POS, IntakeConstants.STOW_TOLERANCE);
    }

    private boolean rawCoralDetected() {
        return inputs.intakeCurrent.gte(IntakeConstants.CORAL_DETECTION_CURRENT);
    }

    public Command toggleCommand(boolean instant) {
        if (goal.lte(IntakeConstants.DEPLOY_TOLERANCE)) {
            return stowCommand(instant);
        } else {
            return deployCommand(instant);
        }
    }

    public Command deployCommand(boolean instant) {
        if (instant) {
            return this.runOnce(() -> setGoal(IntakeConstants.DEPLOY_POS));
        } else {
            return this.runOnce(() -> setGoal(IntakeConstants.DEPLOY_POS)).andThen(Commands.waitUntil(deployedTrigger));
        }
    }

    public Command stowCommand(boolean instant) {
        if (instant) {
            return this.runOnce(this::initiateStow);
        } else {
            return this.runOnce(this::initiateStow).andThen(Commands.waitUntil(stowedTrigger));
        }
    }

    private void initiateStow() {
        goal = IntakeConstants.STOW_POS;
        stowSlowdownActive = true;
        io.setGoal(IntakeConstants.STOW_FAST_POS);
    }

    public void setGuardEnabled(boolean enabled) {
        guardEnabled = enabled;
    }

    public double getSpeedScale() {
        return slowdownScale;
    }

    public Command startIntakeCommand() {
        return Commands.runOnce(() -> {
                    setIntakeSpeed(IntakeConstants.INTAKE_SPEED);
                    setAlignSpeed(IntakeConstants.ALIGN_SPEED);
                })
                .withName("Start Intake Command");
    }

    public Command startEjectCommand() {
        return Commands.runOnce(() -> {
                    setIntakeSpeed(IntakeConstants.EJECT_SPEED);
                    setAlignSpeed(IntakeConstants.EJECT_SPEED);
                })
                .withName("Start Eject Command");
    }

    public Command intakeCommand() {
        return Commands.startEnd(
                        () -> {
                            setIntakeSpeed(IntakeConstants.INTAKE_SPEED);
                            setAlignSpeed(IntakeConstants.ALIGN_SPEED);
                        },
                        () -> {
                            setIntakeSpeed(Volts.zero());
                            setAlignSpeed(Volts.zero());
                        })
                .withName("Intake Command");
    }

    public Command ejectCommand() {
        return Commands.startEnd(
                        () -> {
                            setIntakeSpeed(IntakeConstants.EJECT_SPEED);
                            setAlignSpeed(IntakeConstants.EJECT_SPEED);
                        },
                        () -> {
                            setIntakeSpeed(Volts.zero());
                            setAlignSpeed(Volts.zero());
                        })
                .withName("Eject Command");
    }

    public Command stopIntake() {
        return Commands.runOnce(() -> {
            setIntakeSpeed(Volts.zero());
            setAlignSpeed(Volts.zero());
        });
    }

    private boolean shouldAutoStowForClearance() {
        Pose2d pose = poseSupplier.get();
        if (pose == null) {
            return false;
        }

        Translation2d center = pose.getTranslation();
        Translation2d backPoint = center.plus(new Translation2d(backExtensionMeters, 0)
                .rotateBy(pose.getRotation().plus(Rotation2d.kPi)));

        double centerEdgeClearance = distanceToFieldEdges(center);
        double effectiveClearance = centerEdgeClearance - backExtensionMeters;

        double edgeClearance = distanceToFieldEdges(backPoint);
        Logger.recordOutput("Intake/WallClearanceMeters", edgeClearance);

        if (effectiveClearance < clearanceMeters || edgeClearance < clearanceMeters) {
            return true;
        }

        double stationClearance = distanceToCoralStations(backPoint);
        Logger.recordOutput("Intake/CoralStationClearanceMeters", stationClearance);
        if (stationClearance < clearanceMeters) {
            return true;
        }

        double reefClearance = distanceToNearestReef(backPoint);
        Logger.recordOutput("Intake/ReefClearanceMeters", reefClearance);
        return reefClearance < clearanceMeters;
    }

    private double computeSpeedScale() {
        if (!isDeployed()) {
            return 1.0;
        }
        Pose2d pose = poseSupplier.get();
        if (pose == null) {
            return 1.0;
        }

        Translation2d center = pose.getTranslation();
        double frameHalf = Dimensions.ROBOT_SIZE.div(2).in(Meters);

        double wallClearance = distanceToFieldEdges(center) - frameHalf;
        double reefClearance = distanceToNearestReef(center) - frameHalf;
        double stationClearance = distanceToCoralStations(center) - frameHalf;

        double minClearance = Math.max(0.0, Math.min(wallClearance, Math.min(reefClearance, stationClearance)));

        double maxRange = IntakeConstants.SLOWDOWN_START_DISTANCE.in(Meters);
        double minRange = IntakeConstants.SLOWDOWN_STOP_DISTANCE.in(Meters);

        if (minClearance <= minRange) {
            return 0.0;
        }
        if (minClearance >= maxRange) {
            return 1.0;
        }

        double ratio = MathUtil.clamp((minClearance - minRange) / (maxRange - minRange), 0.0, 1.0);
        return Math.pow(ratio, 2);
    }

    private double distanceToFieldEdges(Translation2d point) {
        double length = VisionConstants.APRIL_TAGS.getFieldLength();
        double width = VisionConstants.APRIL_TAGS.getFieldWidth();
        return Math.min(Math.min(point.getX(), length - point.getX()), Math.min(point.getY(), width - point.getY()));
    }

    private double distanceToCoralStations(Translation2d point) {
        Translation2d[] stations = {
            FieldConstants.LEFT_CORAL_STATION.getTranslation(), FieldConstants.RIGHT_CORAL_STATION.getTranslation()
        };
        double min = Double.POSITIVE_INFINITY;
        for (Translation2d station : stations) {
            double dist = point.getDistance(station);
            if (dist < min) {
                min = dist;
            }
        }
        return min;
    }

    private double distanceToNearestReef(Translation2d point) {
        double reefRadius = FieldConstants.REEF_APOTHEM.in(Meters);
        double distBlue = point.getDistance(FieldConstants.REEF_CENTER_BLUE);
        double distRed = point.getDistance(FieldConstants.REEF_CENTER_RED);
        double nearest = Math.min(distBlue, distRed) - reefRadius;
        return nearest;
    }
}
