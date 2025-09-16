// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.AlignConstants;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.util.ControlConstants;
import java.util.Objects;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

/**
 * Command to align the robot to a specific pose (translation + rotation) using PID controllers. It uses profiled PID
 * controllers for translation and a standard PID controller for rotation.
 */
public class AlignToPoseCommand extends Command {
    private final Supplier<Pose2d> targetSupplier;
    private Pose2d currentTargetPose;
    private final ProfiledPIDController pidControllerX;
    private final ProfiledPIDController pidControllerY;
    private final PIDController pidControllerAngle;
    private final Trigger alignedTrigger;

    private final Swerve swerve;
    private final double translationAngleToleranceDeg;

    /**
     * Aligns the robot to a given pose (translation + rotation).
     *
     * @param targetPose The target pose to align to.
     * @param linearControlConstants The control constants for linear movement (meters).
     * @param angleControlConstants The control constants for angular movement (degrees).
     * @param swerve The swerve subsystem.
     */
    public AlignToPoseCommand(
            Pose2d targetPose,
            ControlConstants linearControlConstants,
            ControlConstants angleControlConstants,
            Swerve swerve) {
        this(targetPose, linearControlConstants, angleControlConstants, swerve, Double.NaN);
    }

    public AlignToPoseCommand(
            Pose2d targetPose,
            ControlConstants linearControlConstants,
            ControlConstants angleControlConstants,
            Swerve swerve,
            double translationAngleToleranceDeg) {
        this(() -> targetPose, linearControlConstants, angleControlConstants, swerve, translationAngleToleranceDeg);
    }

    public AlignToPoseCommand(
            Supplier<Pose2d> targetSupplier,
            ControlConstants linearControlConstants,
            ControlConstants angleControlConstants,
            Swerve swerve,
            double translationAngleToleranceDeg) {
        this.targetSupplier = Objects.requireNonNull(targetSupplier);
        this.swerve = swerve;
        this.translationAngleToleranceDeg = translationAngleToleranceDeg;
        this.currentTargetPose = null;

        pidControllerX = linearControlConstants.getProfiledPIDController();
        pidControllerY = linearControlConstants.getProfiledPIDController();
        pidControllerAngle = angleControlConstants.getPIDController();
        pidControllerAngle.enableContinuousInput(-180, 180);

        alignedTrigger = new Trigger(
                        () -> pidControllerX.atGoal() && pidControllerY.atGoal() && pidControllerAngle.atSetpoint())
                .debounce(AlignConstants.ALIGN_TIME.in(Seconds), DebounceType.kRising);

        addRequirements(swerve);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        // Reset PIDControllers to initial position and velocity
        ChassisSpeeds chassisSpeeds = swerve.getFieldSpeeds();
        pidControllerX.reset(swerve.getPose().getX(), chassisSpeeds.vxMetersPerSecond);
        pidControllerY.reset(swerve.getPose().getY(), chassisSpeeds.vyMetersPerSecond);
        pidControllerAngle.reset();

        updateTargetPose(true);

        Logger.recordOutput("Alignment/Aligned", false);
        Logger.recordOutput("Alignment/TargetPose", currentTargetPose);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        updateTargetPose(false);

        double xOutput = pidControllerX.calculate(swerve.getPose().getX()) + pidControllerX.getSetpoint().velocity;
        double yOutput = pidControllerY.calculate(swerve.getPose().getY()) + pidControllerY.getSetpoint().velocity;
        double currentAngle = swerve.getRotation().getDegrees();
        double omegaOutput = pidControllerAngle.calculate(currentAngle);

        double angleError = MathUtil.inputModulus(pidControllerAngle.getSetpoint() - currentAngle, -180.0, 180.0);
        boolean holdTranslations =
                !Double.isNaN(translationAngleToleranceDeg) && Math.abs(angleError) > translationAngleToleranceDeg;

        LinearVelocity xVel = holdTranslations ? MetersPerSecond.of(0) : MetersPerSecond.of(xOutput);
        LinearVelocity yVel = holdTranslations ? MetersPerSecond.of(0) : MetersPerSecond.of(yOutput);
        AngularVelocity omega = DegreesPerSecond.of(omegaOutput);

        swerve.driveFieldCentric(xVel, yVel, omega);

        Logger.recordOutput(
                "Alignment/Setpoint",
                new Pose2d(
                        pidControllerX.getSetpoint().position,
                        pidControllerY.getSetpoint().position,
                        Rotation2d.fromDegrees(pidControllerAngle.getSetpoint())));
        Logger.recordOutput("Alignment/Distance to Target", getDistanceToTarget());
        if (!Double.isNaN(translationAngleToleranceDeg)) {
            Logger.recordOutput("Alignment/HoldTranslations", holdTranslations);
        }
    }

    public Distance getDistanceToTarget() {
        Pose2d target = currentTargetPose != null ? currentTargetPose : targetSupplier.get();
        return Meters.of(swerve.getPose().getTranslation().getDistance(target.getTranslation()));
    }

    public Trigger withinDistanceToTarget(Distance distance) {
        return new Trigger(() -> getDistanceToTarget().lt(distance));
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        Logger.recordOutput("Alignment/Aligned", !interrupted);
        swerve.brake();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return alignedTrigger.getAsBoolean();
    }

    private void updateTargetPose(boolean force) {
        Pose2d desired = targetSupplier.get();
        if (force || hasTargetChanged(desired)) {
            currentTargetPose = desired;
            pidControllerX.setGoal(desired.getX());
            pidControllerY.setGoal(desired.getY());
            pidControllerAngle.setSetpoint(desired.getRotation().getDegrees());
            if (!force) {
                Logger.recordOutput("Alignment/TargetPose", desired);
            }
        }
    }

    private boolean hasTargetChanged(Pose2d desired) {
        if (currentTargetPose == null) {
            return true;
        }

        double translationDiff = desired.getTranslation()
                .minus(currentTargetPose.getTranslation())
                .getNorm();
        double rotationDiff =
                desired.getRotation().minus(currentTargetPose.getRotation()).getDegrees();
        return translationDiff > 1e-3 || Math.abs(rotationDiff) > 0.1;
    }
}
