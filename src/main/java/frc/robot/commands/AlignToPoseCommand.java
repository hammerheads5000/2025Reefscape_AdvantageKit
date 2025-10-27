// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.AlignConstants;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.util.ControlConstants;
import org.littletonrobotics.junction.Logger;

/**
 * Command to align the robot to a specific pose (translation + rotation) using PID controllers. It uses profiled PID
 * controllers for translation and a standard PID controller for rotation.
 */
public class AlignToPoseCommand extends Command {
    public final Pose2d targetPose;
    private final ProfiledPIDController pidControllerX;
    private final ProfiledPIDController pidControllerY;
    private final SimpleMotorFeedforward feedforwardX;
    private final SimpleMotorFeedforward feedforwardY;
    private final PIDController pidControllerAngle;
    private final Trigger alignedTrigger;

    private final Swerve swerve;

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
        this.targetPose = targetPose;
        this.swerve = swerve;

        pidControllerX = linearControlConstants.getProfiledPIDController();
        pidControllerY = linearControlConstants.getProfiledPIDController();
        pidControllerAngle = angleControlConstants.getPIDController();
        pidControllerAngle.enableContinuousInput(-180, 180);

        pidControllerX.setGoal(0);
        pidControllerY.setGoal(0);
        pidControllerAngle.setSetpoint(targetPose.getRotation().getDegrees());

        feedforwardX = linearControlConstants.getSimpleFeedforward();
        feedforwardY = linearControlConstants.getSimpleFeedforward();

        alignedTrigger = new Trigger(
                        () -> pidControllerX.atGoal() && pidControllerY.atGoal() && pidControllerAngle.atSetpoint())
                .debounce(AlignConstants.ALIGN_TIME.in(Seconds), DebounceType.kRising);

        addRequirements(swerve);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        // Reset PIDControllers to initial position and velocity
        ChassisSpeeds chassisSpeeds = getRelativeSpeeds(swerve.getFieldSpeeds());
        pidControllerX.reset(getRelativePose().getX(), chassisSpeeds.vxMetersPerSecond);
        pidControllerY.reset(getRelativePose().getY(), chassisSpeeds.vyMetersPerSecond);
        pidControllerAngle.reset();

        pidControllerX.setGoal(0);
        pidControllerY.setGoal(0);
        pidControllerAngle.setSetpoint(targetPose.getRotation().getDegrees());

        Logger.recordOutput("Alignment/Aligned", false);
        Logger.recordOutput("Alignment/TargetPose", targetPose);
    }

    private ChassisSpeeds getRelativeSpeeds(ChassisSpeeds fieldSpeeds) {
        return ChassisSpeeds.fromFieldRelativeSpeeds(fieldSpeeds, targetPose.getRotation());
    }

    private ChassisSpeeds getFieldSpeeds(ChassisSpeeds relativeSpeeds) {
        return ChassisSpeeds.fromRobotRelativeSpeeds(relativeSpeeds, targetPose.getRotation());
    }

    private Pose2d getRelativePose() {
        return swerve.getPose().relativeTo(targetPose);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        Pose2d relativePose = getRelativePose();
        double xVel = pidControllerX.calculate(relativePose.getX())
                + feedforwardX.calculate(pidControllerX.getSetpoint().velocity);
        double yVel = pidControllerY.calculate(relativePose.getY())
                + feedforwardY.calculate(pidControllerY.getSetpoint().velocity);
        double omega = DegreesPerSecond.of(
                        pidControllerAngle.calculate(swerve.getRotation().getDegrees()))
                .in(RadiansPerSecond);

        // target pose relative -> field relative -> robot relative
        ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                getFieldSpeeds(new ChassisSpeeds(xVel, yVel, omega)), swerve.getRotation());

        swerve.drive(speeds);

        Pose2d setpoint = targetPose.transformBy(new Transform2d(
                new Translation2d(pidControllerX.getSetpoint().position, pidControllerY.getSetpoint().position),
                Rotation2d.kZero));

        Logger.recordOutput(
                "Alignment/Setpoint (field relative)",
                new Pose2d(setpoint.getX(), setpoint.getY(), Rotation2d.fromDegrees(pidControllerAngle.getSetpoint())));
        Logger.recordOutput("Alignment/setpointX", pidControllerX.getSetpoint().position);
        Logger.recordOutput("Alignment/setpointY", pidControllerY.getSetpoint().position);
        Logger.recordOutput("Alignment/measuredX", relativePose.getX());
        Logger.recordOutput("Alignment/measuredY", relativePose.getY());
        Logger.recordOutput("Alignment/Distance to Target", getDistanceToTarget());
    }

    public Distance getDistanceToTarget() {
        return Meters.of(getRelativePose().getTranslation().getNorm());
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
}
