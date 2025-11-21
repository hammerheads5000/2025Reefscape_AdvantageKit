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
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.AlignConstants;
import frc.robot.Constants.Dimensions;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.PathConstants;
import frc.robot.subsystems.coraldetection.CoralDetection;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.endeffector.EndEffector;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.util.BoundaryProtections;
import frc.robot.util.SlewRateLimiter2d;
import frc.robot.util.TunablePIDController;
import java.util.Set;

/** Ends right after coral detected, without stopping */
public class AutoCoralCommand extends SequentialCommandGroup {
    private final Swerve swerve;
    private final Intake intake;
    private final EndEffector endEffector;
    private final Elevator elevator;
    private final CoralDetection coralDetection;

    private SlewRateLimiter2d accelerationLimiter = new SlewRateLimiter2d(2);
    private TunablePIDController rotationController =
            new TunablePIDController(AlignConstants.CORAL_PICKUP_ANGLE); // in radians

    private Distance distanceToCoral = Meters.of(100); // default very far away
    private final boolean ignoreWall;

    public AutoCoralCommand(
            Swerve swerve, Intake intake, EndEffector endEffector, Elevator elevator, CoralDetection coralDetection) {
        this(swerve, intake, endEffector, elevator, coralDetection, false);
    }

    public AutoCoralCommand(
            Swerve swerve,
            Intake intake,
            EndEffector endEffector,
            Elevator elevator,
            CoralDetection coralDetection,
            boolean ignoreWall) {
        this.swerve = swerve;
        this.intake = intake;
        this.endEffector = endEffector;
        this.elevator = elevator;
        this.coralDetection = coralDetection;
        this.ignoreWall = ignoreWall;

        addCommands(
                this.intake.deployCommand(true),
                this.intake.startIntakeCommand(),
                this.endEffector.startIntakeCommand(),
                Commands.defer(this::driveTowardsCoral, Set.of(swerve))
                        .withTimeout(IntakeConstants.CORAL_TIMEOUT)
                        .repeatedly() // will keep restarting after timeout until coral detected in intake
                        .until(intake.coralDetectedTrigger.or(endEffector.coralDetectedTrigger)),
                Commands.either( // depending on elevator being at intake or not
                        this.intake.startSlowIntakeCommand(),
                        new ScheduleCommand(Commands.waitSeconds(0.1).andThen(this.intake.stopIntake())),
                        () -> elevator.getHeight()
                                .isNear(ElevatorConstants.INTAKE_HEIGHT, ElevatorConstants.TOLERANCE)),
                Commands.waitSeconds(0.05),
                Commands.runOnce(() -> this.swerve.stop()));
    }

    private Command driveTowardsCoral() {
        // If the coral is next to a wall, use AlignToPoseCommand to not slam into wall
        Translation2d coral = coralDetection.getClosestCoral(ignoreWall);
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
                            Translation2d closestCoral = coralDetection.getClosestCoral(ignoreWall);
                            if (closestCoral == null) {
                                // Translation2d vel = accelerationLimiter.calculate(new Translation2d());
                                swerve.driveFieldCentric(
                                        MetersPerSecond.zero(), MetersPerSecond.zero(), RadiansPerSecond.zero());
                                return;
                            }

                            // moving towards coral
                            Translation2d vel =
                                    closestCoral.minus(swerve.getPose().getTranslation());
                            distanceToCoral = Meters.of(vel.getNorm());
                            vel = vel.div(vel.getNorm());
                            vel = vel.times(IntakeConstants.PICKUP_SPEED.in(MetersPerSecond));
                            vel = accelerationLimiter.calculate(vel);

                            AngularVelocity omega = RadiansPerSecond.of(rotationController.calculate(
                                    swerve.getPose().getRotation().getRadians(),
                                    vel.getAngle().plus(Rotation2d.k180deg).getRadians()));

                            // don't run into wall (hopefully)
                            vel = BoundaryProtections.adjustVelocity(swerve.getPose(), vel);

                            swerve.driveFieldCentric(
                                    vel.getMeasureX().per(Second),
                                    vel.getMeasureY().per(Second),
                                    omega);
                        },
                        swerve)
                .until(() -> coralDetection.getClosestCoral() != coralDetection.getClosestCoral(true));
    }

    /** use PathPlanner to create path to pickup coral on a wall */
    private Command pathfindToCoralCommand(Translation2d coralPose, Pose2d nearestBoundaryPose) {
        Pose2d pose = swerve.getPose();
        double endRobotDistToWall = Dimensions.ROBOT_SIZE.in(Meters) / 2
                + IntakeConstants.INTAKE_EXTENSION.in(Meters)
                + IntakeConstants.DISTANCE_TO_KEEP_FROM_WALL.in(Meters);

        Pose2d pickupPose = nearestBoundaryPose.transformBy(
                new Transform2d(new Translation2d(endRobotDistToWall, 0), Rotation2d.k180deg));
        Pose2d approachPose = pickupPose.transformBy(new Transform2d(
                new Translation2d(-PathConstants.CORAL_APPROACH_DISTANCE.in(Meters), 0), Rotation2d.kZero));

        PathPlannerPath path = new PathPlannerPath(
                PathPlannerPath.waypointsFromPoses(
                        pose.rotateAround(pose.getTranslation(), Rotation2d.k180deg), approachPose, pickupPose),
                PathConstants.FAST_CONSTRAINTS,
                new IdealStartingState(0, Rotation2d.kZero),
                new GoalEndState(0, pickupPose.getRotation().rotateBy(Rotation2d.k180deg)));

        path.preventFlipping = true;

        distanceToCoral = Meters.zero(); // run intake immediately

        return AutoBuilder.followPath(path)
                .andThen(new AlignToPoseCommand(
                        approachPose.rotateAround(approachPose.getTranslation(), Rotation2d.k180deg),
                        AlignConstants.APPROACH_TRANSLATION,
                        AlignConstants.APPROACH_ANGLE,
                        swerve));
    }
}
