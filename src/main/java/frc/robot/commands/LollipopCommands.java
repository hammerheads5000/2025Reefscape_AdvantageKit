// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.AlignConstants;
import frc.robot.Constants.PathConstants;
import frc.robot.FieldConstants;
import frc.robot.subsystems.algaemanipulator.AlgaeManipulator;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.util.FlipUtil;
import java.util.Arrays;

/** Container class to pick up algae from lollipops (those things with alge on top of coral). */
public class LollipopCommands {
    private static Command lollipopCommand(
            Translation2d lollipopPos, Swerve swerve, AlgaeManipulator algaeManipulator, Elevator elevator) {
        lollipopPos = FlipUtil.applyAlliance(lollipopPos);
        Pose2d lollipopPose = new Pose2d(
                lollipopPos,
                Pathfinding.pointPoseTowards(swerve.getPose(), lollipopPos).getRotation());
        AlignToPoseCommand align = new AlignToPoseCommand(
                lollipopPose, AlignConstants.LOLLIPOP_PID_TRANSLATION, AlignConstants.LOLLIPOP_PID_ANGLE, swerve);
        return Commands.parallel(
                        elevator.goToLollipopCommand(true),
                        align,
                        Commands.waitUntil(align.withinDistanceToTarget(PathConstants.LOLLIPOP_INTAKE_DISTANCE))
                                .andThen(algaeManipulator.intakeCommand()))
                .withName("Lollipop");
    }

    /**
     * Command to align to a given lollipop and intake algae from it.
     *
     * @param lollipop the index of the lollipop to align to (0-3 right to left)
     * @param swerve
     * @param algaeManipulator
     * @param elevator
     * @return Lollipop command
     */
    public static Command lollipopCommand(
            int lollipop, Swerve swerve, AlgaeManipulator algaeManipulator, Elevator elevator) {
        Translation2d lollipopPose = FieldConstants.LOLLIPOP_POSES[lollipop];
        return lollipopCommand(lollipopPose, swerve, algaeManipulator, elevator);
    }

    /**
     * Command to align to the closest lollipop and intake algae from it.
     *
     * @param swerve
     * @param algaeManipulator
     * @param elevator
     * @return Lollipop command
     */
    public static Command lollipopCommand(Swerve swerve, AlgaeManipulator algaeManipulator, Elevator elevator) {
        Translation2d robotTranslation = FlipUtil.applyAlliance(swerve.getPose().getTranslation());
        Translation2d closest = robotTranslation.nearest(Arrays.asList(FieldConstants.LOLLIPOP_POSES));

        return lollipopCommand(closest, swerve, algaeManipulator, elevator);
    }

    private LollipopCommands() {
        // Prevent instantiation
    }
}
