// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Second;

import java.util.Set;

import edu.wpi.first.math.controller.PIDController;
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
import frc.robot.subsystems.coraldetection.CoralDetection;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.endeffector.EndEffector;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.swerve.Swerve;
import java.util.Optional;
import java.util.Set;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AutoCoralCommand extends SequentialCommandGroup {
    private final Swerve swerve;
    private final Intake intake;
    private final EndEffector endEffector;
    private final Elevator elevator;
    private final CoralDetection coralDetection;

    private SlewRateLimiter2d accelerationLimiter = new SlewRateLimiter2d(2);
    private PIDController rotationController = AlignConstants.CORAL_PICKUP_PID_ANGLE.getPIDController();

    public AutoCoralCommand(
            Swerve swerve, Intake intake, EndEffector endEffector, Elevator elevator, CoralDetection coralDetection) {
        this.swerve = swerve;
        this.intake = intake;
        this.endEffector = endEffector;
        this.elevator = elevator;
        this.coralDetection = coralDetection;
        this.elevator = elevator;

        addRequirements(swerve, intake, endEffector, elevator);

        addCommands();
    }

    private Command driveTowardsCoral() {
        return Commands.run(() -> {
            Translation2d closestCoral = coralDetection.getClosestCoral();
            if (closestCoral == null) {
                Translation2d vel = accelerationLimiter.calculate(new Translation2d());
                swerve.driveFieldCentric(vel.getMeasureX().per(Second), vel.getMeasureY().per(Second), RadiansPerSecond.zero());
                return;
            }

            Translation2d vel = closestCoral.minus(swerve.getPose().getTranslation());
            vel = vel.div(vel.getNorm());
            vel = vel.times(AlignConstants.CORAL_APPROACH_SPEED);
        }, Set.of(swerve));
    }
}
