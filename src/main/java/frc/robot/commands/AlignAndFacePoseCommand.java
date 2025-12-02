// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Seconds;

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
import frc.robot.util.TunableControls.TunableControlConstants;
import frc.robot.util.TunableControls.TunablePIDController;
import frc.robot.util.TunableControls.TunableProfiledController;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

/**
 * Command to align the robot to a specific pose (translation + rotation) using PID controllers. It uses profiled PID
 * controllers for translation and a standard PID controller for rotation.
 */
public class AlignAndFacePoseCommand extends Command {
    public final Pose2d targetPose;
    private Supplier<Pose2d> poseToFaceSupplier;
    private Pose2d poseToFace;
    private final TunableProfiledController pidControllerX;
    private final TunableProfiledController pidControllerY;
    private final TunablePIDController pidControllerAngle;
    private final Trigger alignedTrigger;

    private final Swerve swerve;

    /**
     * Aligns the robot to a given pose (translation) while facing supplied pose.
     *
     * @param targetPose The target pose to align to.
     * @param defaultPoseToFace The default pose to face if the supplier returns null.
     * @param poseToFaceSupplier A supplier that provides the pose to face. If it returns null, the defaultPoseToFace is
     *     used.
     * @param linearControlConstants The control constants for linear movement (meters).
     * @param angleControlConstants The control constants for angular movement (degrees).
     * @param swerve The swerve subsystem.
     */
    public AlignAndFacePoseCommand(
            Pose2d targetPose,
            Pose2d defaultPoseToFace,
            Supplier<Pose2d> poseToFaceSupplier,
            TunableControlConstants linearControlConstants,
            TunableControlConstants angleControlConstants,
            Swerve swerve) {
        this.targetPose = targetPose;
        this.poseToFaceSupplier = poseToFaceSupplier;
        this.poseToFace = defaultPoseToFace;
        this.swerve = swerve;

        pidControllerX = new TunableProfiledController(linearControlConstants);
        pidControllerY = new TunableProfiledController(linearControlConstants);
        pidControllerAngle = new TunablePIDController(angleControlConstants);

        pidControllerX.setGoal(targetPose.getX());
        pidControllerY.setGoal(targetPose.getY());
        pidControllerAngle.setSetpoint(targetPose.getRotation().getDegrees());

        alignedTrigger = new Trigger(
                        () -> pidControllerX.atGoal() && pidControllerY.atGoal() && pidControllerAngle.atSetpoint())
                .debounce(AlignConstants.ALIGN_TIME.in(Seconds), DebounceType.kRising);

        addRequirements(swerve);
    }

    /**
     * Aligns the robot to a given pose (translation) while facing given pose.
     *
     * @param targetPose The target pose to align to.
     * @param poseToFace The pose to face.
     * @param linearControlConstants The control constants for linear movement (meters).
     * @param angleControlConstants The control constants for angular movement (degrees).
     * @param swerve The swerve subsystem.
     */
    public AlignAndFacePoseCommand(
            Pose2d targetPose,
            Pose2d poseToFace,
            TunableControlConstants linearControlConstants,
            TunableControlConstants angleControlConstants,
            Swerve swerve) {
        this(targetPose, poseToFace, () -> poseToFace, linearControlConstants, angleControlConstants, swerve);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        // Reset PIDControllers to initial position and velocity
        ChassisSpeeds chassisSpeeds = swerve.getFieldSpeeds();
        pidControllerX.reset(swerve.getPose().getX(), chassisSpeeds.vxMetersPerSecond);
        pidControllerY.reset(swerve.getPose().getY(), chassisSpeeds.vyMetersPerSecond);
        pidControllerAngle.reset();

        pidControllerX.setGoal(targetPose.getX());
        pidControllerY.setGoal(targetPose.getY());
        pidControllerAngle.setSetpoint(Pathfinding.pointPoseTowards(swerve.getPose(), poseToFace)
                .getRotation()
                .getDegrees());

        Logger.recordOutput("Alignment/Aligned", false);
        Logger.recordOutput("Alignment/TargetPose", targetPose);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        processPoseToFace();

        pidControllerAngle.setSetpoint(Pathfinding.pointPoseTowards(swerve.getPose(), poseToFace)
                .getRotation()
                .getDegrees());

        LinearVelocity xVel =
                MetersPerSecond.of(pidControllerX.calculate(swerve.getPose().getX()));
        LinearVelocity yVel =
                MetersPerSecond.of(pidControllerY.calculate(swerve.getPose().getY()));
        AngularVelocity omega = DegreesPerSecond.of(
                pidControllerAngle.calculate(swerve.getRotation().getDegrees()));

        swerve.driveFieldCentric(xVel, yVel, omega);

        Logger.recordOutput(
                "Alignment/Setpoint",
                new Pose2d(
                        pidControllerX.getSetpoint().position,
                        pidControllerY.getSetpoint().position,
                        Rotation2d.fromDegrees(pidControllerAngle.getSetpoint())));

        Logger.recordOutput("Alignment/Distance to Target", getDistanceToTarget());

        Logger.recordOutput("Alignment/Error/xP", pidControllerX.getPositionError());
        Logger.recordOutput("Alignment/Error/xI", pidControllerX.getAccumulatedError());
        Logger.recordOutput("Alignment/Error/xD", pidControllerX.getVelocityError());

        Logger.recordOutput("Alignment/Error/yP", pidControllerY.getPositionError());
        Logger.recordOutput("Alignment/Error/yI", pidControllerY.getAccumulatedError());
        Logger.recordOutput("Alignment/Error/yD", pidControllerY.getVelocityError());

        Logger.recordOutput("Alignment/Error/angleP", pidControllerAngle.getPositionError());
        Logger.recordOutput("Alignment/Error/angleI", pidControllerAngle.getAccumulatedError());
    }

    public Distance getDistanceToTarget() {
        return Meters.of(swerve.getPose().getTranslation().getDistance(targetPose.getTranslation()));
    }

    public Trigger withinDistanceToTarget(Distance distance) {
        return new Trigger(() -> getDistanceToTarget().lt(distance));
    }

    public Trigger alignedAngle() {
        return new Trigger(pidControllerAngle::atSetpoint);
    }

    private void processPoseToFace() {
        Pose2d pose = poseToFaceSupplier.get();
        if (pose != null) {
            poseToFace = pose;
        }
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
