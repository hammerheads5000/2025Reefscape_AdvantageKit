// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.coraldetection.CoralDetection;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.endeffector.EndEffector;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.util.SlewRateLimiter2d;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AutoCoralCommand extends SequentialCommandGroup {
    Swerve swerve;
    Intake intake;
    EndEffector endEffector;
    Elevator elevator;
    CoralDetection coralDetection;
    PIDController pid = new PIDController(0.1, 0, 0);
    SlewRateLimiter2d slewRateLimiter = new SlewRateLimiter2d(3);

    public AutoCoralCommand(
            Swerve swerve, Intake intake, EndEffector endEffector, Elevator elevator, CoralDetection coralDetection) {
        this.swerve = swerve;
        this.intake = intake;
        this.endEffector = endEffector;
        this.elevator = elevator;
        this.coralDetection = coralDetection;

        addCommands(
                Commands.runOnce(() -> slewRateLimiter.calculate(
                        swerve.getFieldSpeeds().vxMetersPerSecond, swerve.getFieldSpeeds().vyMetersPerSecond)),
                elevator.goToIntakePosCommand(false),
                intake.deployCommand(false),
                intake.startIntakeCommand(),
                endEffector.intakeCommand().deadlineFor(mainCommand().until(intake.coralDetectedTrigger)),
                intake.stopIntake());
    }

    private Command mainCommand() {
        return Commands.run(
                () -> {
                    var closestCoral = coralDetection.getClosestCoral();
                    if (closestCoral == null) {
                        swerve.driveFieldCentric(
                                MetersPerSecond.zero(), MetersPerSecond.zero(), RadiansPerSecond.zero());
                        return;
                    }
                    Rotation2d targetAngle = new Rotation2d(
                                    VisionConstants.CORAL_CAM_POS.getRotation().getZ())
                            .minus(Rotation2d.fromDegrees(closestCoral.getX()))
                            .rotateBy(Rotation2d.kPi);
                    System.out.println("HI");
                    Translation2d driveVec = new Translation2d(-IntakeConstants.PICKUP_SPEED.in(MetersPerSecond), 0)
                            .rotateBy(targetAngle);
                    driveVec = slewRateLimiter.calculate(driveVec);
                    System.out.println("HIII");
                    swerve.driveFieldCentric(
                            MetersPerSecond.of(driveVec.getX()),
                            MetersPerSecond.of(driveVec.getY()),
                            RadiansPerSecond.of(pid.calculate(
                                    targetAngle.getRadians(),
                                    swerve.getRotation().getRadians())));
                    System.out.println("HIIIIIIIII");
                },
                swerve);
    }
}
