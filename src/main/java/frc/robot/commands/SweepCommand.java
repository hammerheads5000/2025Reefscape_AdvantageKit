// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.AlignConstants;
import frc.robot.Constants.PathConstants;
import frc.robot.subsystems.swerve.Swerve;

/** Sweep the nearest reef side to remove coral from being in the way */
public class SweepCommand extends SequentialCommandGroup {
    /** Creates a new SweepCommand. */
    public SweepCommand(Swerve swerve) {
        Pose2d pose = swerve.getPose();
        int side = Pathfinding.getClosestReefSide(swerve.getPose());
        Pose2d leftPose = AlignToReefCommands.getReefPose(side, PathConstants.SWEEP_RELATIVE_POS);
        Pose2d rightPose = AlignToReefCommands.getReefPose(side, -PathConstants.SWEEP_RELATIVE_POS);
        Pose2d centerPose = AlignToReefCommands.getReefPose(side, 0);
        Pose2d pose1;
        if (leftPose.getTranslation().getDistance(pose.getTranslation())
                < rightPose.getTranslation().getDistance(pose.getTranslation())) {
            pose1 = leftPose;
        } else {
            pose1 = rightPose;
        }
        pose1 = pose1.transformBy(new Transform2d(
                new Translation2d(PathConstants.SWEEP_OFFSET.unaryMinus(), Meters.zero()), Rotation2d.kZero));
        addCommands(
                new AlignToPoseCommand(
                        pose1, AlignConstants.SWEEP_TRANSLATION, AlignConstants.SWEEP_ANGLE, swerve),
                new AlignToPoseCommand(
                        centerPose, AlignConstants.SWEEP_TRANSLATION, AlignConstants.SWEEP_ANGLE, swerve));
    }
}
