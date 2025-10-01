// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Second;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.IdealStartingState;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Constants.AlignConstants;
import frc.robot.Constants.Dimensions;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.PathConstants;
import frc.robot.subsystems.coraldetection.CoralDetection;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.endeffector.EndEffector;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.util.BoundaryProtections;
import frc.robot.util.SlewRateLimiter2d;
import java.util.Set;

/** Ends right after coral detected, without stopping */
public class AutoCoralCommand extends ParallelCommandGroup {
    private final Swerve swerve;
    private final Intake intake;
    private final EndEffector endEffector;
    private final Elevator elevator;
    private final CoralDetection coralDetection;

    private SlewRateLimiter2d accelerationLimiter = new SlewRateLimiter2d(2);
    private PIDController rotationController = AlignConstants.CORAL_PICKUP_PID_ANGLE.getPIDController(); // in radians

    private Distance distanceToCoral = Meters.of(100); // default very far away

    public AutoCoralCommand(
            Swerve swerve, Intake intake, EndEffector endEffector, Elevator elevator, CoralDetection coralDetection) {
        this.swerve = swerve;
        this.intake = intake;
        this.endEffector = endEffector;
        this.elevator = elevator;
        this.coralDetection = coralDetection;

        rotationController.enableContinuousInput(0, 2 * Math.PI);

        addCommands(
                Commands.sequence( // zero the elevator at the start, but don't wait for it
                        this.elevator.goToIntakePosCommand(false),
                        this.elevator.zeroEncoderCommand(),
                        this.elevator.goToIntakePosCommand(true)),
                Commands.sequence(
                        Commands.waitUntil(() -> distanceToCoral.lt(IntakeConstants.START_DISTANCE)),
                        this.intake.startIntakeCommand()),
                Commands.sequence(
                        this.intake.deployCommand(true),
                        Commands.defer(this::driveTowardsCoral, Set.of(swerve))
                                .withTimeout(IntakeConstants.CORAL_TIMEOUT)
                                .repeatedly() // will keep restarting after timeout until coral detected in intake
                                .until(intake.coralDetectedTrigger.or(endEffector.coralDetectedTrigger)),
                        this.intake.startSlowIntakeCommand(),
                        this.endEffector.startIntakeCommand(),
                        Commands.waitSeconds(0.05),
                        Commands.runOnce(() -> this.swerve.stop())));
    }

    private Command driveTowardsCoral() {
        // If the coral is next to a wall, use AlignToPoseCommand to not slam into wall
        Translation2d coral = coralDetection.getClosestCoral();
        Pose2d nearestBoundaryPose;
        if (coral != null
                && (nearestBoundaryPose = BoundaryProtections.nearestBoundaryPose(coral))
                                .getTranslation()
                                .getDistance(coral)
                        < IntakeConstants.CORAL_ON_WALL_THRESHOLD.in(Meters)) {
            return pathfindToCoralCommand(coral, nearestBoundaryPose);
        }
        return Commands.run(
                () -> {
                    Translation2d closestCoral = coralDetection.getClosestCoral();
                    if (closestCoral == null) {
                        // Translation2d vel = accelerationLimiter.calculate(new Translation2d());
                        swerve.driveFieldCentric(
                                MetersPerSecond.zero(), MetersPerSecond.zero(), RadiansPerSecond.zero());
                        return;
                    }

                    Translation2d vel = closestCoral.minus(swerve.getPose().getTranslation());
                    distanceToCoral = Meters.of(vel.getNorm());
                    vel = vel.div(vel.getNorm());
                    vel = vel.times(IntakeConstants.PICKUP_SPEED.in(MetersPerSecond));
                    vel = accelerationLimiter.calculate(vel);

                    AngularVelocity omega = RadiansPerSecond.of(rotationController.calculate(
                            swerve.getPose().getRotation().getRadians(),
                            vel.getAngle().plus(Rotation2d.k180deg).getRadians()));

                    vel = BoundaryProtections.adjustVelocity(swerve.getPose(), vel);

                    swerve.driveFieldCentric(
                            vel.getMeasureX().per(Second), vel.getMeasureY().per(Second), omega);
                },
                swerve);
    }

    private Command pathfindToCoralCommand(Translation2d coralPose, Pose2d nearestBoundaryPose) {
        Pose2d pose = swerve.getPose();
        double endRobotDistToWall = Dimensions.ROBOT_SIZE.in(Meters) / 2 + IntakeConstants.INTAKE_EXTENSION.in(Meters);

        Pose2d pickupPose = nearestBoundaryPose.transformBy(
                new Transform2d(new Translation2d(endRobotDistToWall, 0), Rotation2d.k180deg));
        Pose2d approachPose = pickupPose.transformBy(new Transform2d(
                new Translation2d(-PathConstants.CORAL_APPROACH_DISTANCE.in(Meters), 0), Rotation2d.kZero));

        PathPlannerPath path = new PathPlannerPath(
                PathPlannerPath.waypointsFromPoses(
                        pose.rotateAround(pose.getTranslation(), Rotation2d.k180deg), approachPose, pickupPose),
                PathConstants.APPROACH_CONSTRAINTS,
                new IdealStartingState(0, Rotation2d.kZero),
                new GoalEndState(0, pickupPose.getRotation().rotateBy(Rotation2d.k180deg)));

        path.preventFlipping = true;

        Command updateCoralDistanceCommand = Commands.run(() -> {
            distanceToCoral = Meters.of(BoundaryProtections.nearestBoundaryPose(pose.getTranslation())
                    .getTranslation()
                    .getDistance(swerve.getPose().getTranslation()));
        });

        return AutoBuilder.followPath(path)
                .deadlineFor(updateCoralDistanceCommand)
                .andThen(new AlignToPoseCommand(
                        approachPose.rotateAround(approachPose.getTranslation(), Rotation2d.k180deg),
                        AlignConstants.CORAL_PULL_PID_TRANSLATION,
                        AlignConstants.CORAL_PULL_PID_ANGLE,
                        swerve));
    }
}
