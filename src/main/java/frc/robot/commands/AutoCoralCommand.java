// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Second;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.AlignConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.coraldetection.CoralDetection;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.endeffector.EndEffector;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.util.SlewRateLimiter2d;
import java.util.Set;

/** Ends right after coral detected, without stopping */
public class AutoCoralCommand extends SequentialCommandGroup {
    private final Swerve swerve;
    private final Intake intake;
    private final EndEffector endEffector;
    private final Elevator elevator;
    private final CoralDetection coralDetection;

    private SlewRateLimiter2d accelerationLimiter = new SlewRateLimiter2d(2);
    private PIDController rotationController = AlignConstants.CORAL_PICKUP_PID_ANGLE.getPIDController(); // in radians

    public AutoCoralCommand(
            Swerve swerve, Intake intake, EndEffector endEffector, Elevator elevator, CoralDetection coralDetection) {
        this.swerve = swerve;
        this.intake = intake;
        this.endEffector = endEffector;
        this.elevator = elevator;
        this.coralDetection = coralDetection;

        rotationController.enableContinuousInput(0, 2 * Math.PI);

        addRequirements(swerve, intake, endEffector, elevator);

        addCommands(
                this.elevator.goToIntakePosCommand(true),
                this.intake.deployCommand(true),
                this.intake.startIntakeCommand(),
                this.endEffector.startIntakeCommand(),
                Commands.defer(this::driveTowardsCoral, Set.of(swerve))
                        .withTimeout(IntakeConstants.CORAL_TIMEOUT)
                        .repeatedly() // will keep restarting after timeout until coral detected in intake
                        .until(intake.coralDetectedTrigger.or(endEffector.coralDetectedTrigger)));
    }

    private Command driveTowardsCoral() {
        // If the coral is next to a wall, use AlignToPoseCommand to not slam into wall
        // Translation2d coral = coralDetection.getClosestCoral();
        // if (coral != null && isCoralNextToWall(coral)) {
        //     Translation2d dir = coral.minus(swerve.getPose().getTranslation());
        //     return new AlignToPoseCommand(
        //             new Pose2d(coral, dir.getAngle()),
        //             AlignConstants.CORAL_PICKUP_PID_TRANSLATION,
        //             AlignConstants.CORAL_PICKUP_PID_ANGLE,
        //             swerve);
        // }
        return Commands.run(
                () -> {
                    Translation2d closestCoral = coralDetection.getClosestCoral();
                    if (closestCoral == null) {
                        Translation2d vel = accelerationLimiter.calculate(new Translation2d());
                        swerve.driveFieldCentric(
                                vel.getMeasureX().per(Second), vel.getMeasureY().per(Second), RadiansPerSecond.zero());
                        return;
                    }

                    Translation2d vel = closestCoral.minus(swerve.getPose().getTranslation());
                    vel = vel.div(vel.getNorm());
                    vel = vel.times(IntakeConstants.PICKUP_SPEED.in(MetersPerSecond));
                    vel = accelerationLimiter.calculate(vel);

                    AngularVelocity omega = RadiansPerSecond.of(rotationController.calculate(
                            swerve.getPose().getRotation().getRadians(),
                            vel.getAngle().plus(Rotation2d.k180deg).getRadians()));

                    swerve.driveFieldCentric(
                            vel.getMeasureX().per(Second), vel.getMeasureY().per(Second), omega);
                },
                swerve);
    }
}
