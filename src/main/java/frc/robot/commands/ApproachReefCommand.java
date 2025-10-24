// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static edu.wpi.first.units.Units.Meters;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.PathConstants;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.vision.Vision;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

/** Follow generated path to reef if necessary, then align */
public class ApproachReefCommand extends SequentialCommandGroup {
    private final AlignToPoseCommand alignToReefCommand;
    private final Translation2d reefPos;
    private Swerve swerve;
    private boolean runningPath = true;
    /** Creates a new ApproachReefCommand. */
    public ApproachReefCommand(int side, double relativePos, Swerve swerve, Elevator elevator, Vision vision) {
        this.swerve = swerve;
        alignToReefCommand = AlignToReefCommands.alignToReef(side, relativePos, swerve);

        // don't generate path if too short
        // if (alignToReefCommand
        //         .withinDistanceToTarget(PathConstants.MIN_PATH_DISTANCE)
        //         .getAsBoolean()) {
        //     addCommands(alignToReefCommand);
        //     return;
        // }

        PathPlannerPath path =
                Pathfinding.generateReefPath(swerve.getPose(), side, relativePos, swerve.getFieldSpeeds());
        Translation2d approachPos =
                path.getWaypoints().get(path.getWaypoints().size() - 2).anchor();

        Command followPathCommand = AutoBuilder.followPath(path);

        this.reefPos = alignToReefCommand
                .targetPose
                .transformBy(new Transform2d(
                        new Translation2d(PathConstants.OFFSET_FROM_REEF, Meters.zero()), Rotation2d.kZero))
                .getTranslation();

        Logger.recordOutput("ApproachReef/FinishedPath", false);
        Logger.recordOutput(
                "ApproachReef/ApproachPose", new Pose2d(approachPos, alignToReefCommand.targetPose.getRotation()));

        addCommands(
                followPathCommand
                        .handleInterrupt(() -> alignToReefCommand.end(true))
                        .until(() -> swerve.getPose().getTranslation().getDistance(approachPos)
                                <= PathConstants.APPROACH_TOLERANCE.in(Meters)),
                Commands.runOnce(() -> {
                    this.runningPath = false;
                    Logger.recordOutput("ApproachReef/FinishedPath", true);
                }),
                alignToReefCommand);
    }

    public Distance getDistanceToTarget() {
        Logger.recordOutput(
                "ApproachReef/DistanceToReef",
                Meters.of(swerve.getPose().getTranslation().getDistance(reefPos)));
        return Meters.of(swerve.getPose().getTranslation().getDistance(reefPos));
    }

    public Trigger withinRangeTrigger(Distance range) {
        return alignToReefCommand.withinDistanceToTarget(range);
    }

    @AutoLogOutput
    public Trigger finishedPath() {
        return new Trigger(() -> !this.runningPath);
    }
}
