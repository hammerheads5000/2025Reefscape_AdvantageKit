// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.vision.Vision;

/** Follow generated path to reef if necessary, then align */
public class ApproachReefCommand extends SequentialCommandGroup {
    private final AlignAndFacePoseCommand alignToReefCommand;
    /** Creates a new ApproachReefCommand. */
    public ApproachReefCommand(int side, double relativePos, Swerve swerve, Elevator elevator, Vision vision) {
        alignToReefCommand = AlignToReefCommands.alignToReefFacingBranch(side, relativePos, swerve);

        // don't generate path if too short
        // if (alignToReefCommand
        //         .withinDistanceToTarget(PathConstants.MIN_PATH_DISTANCE)
        //         .getAsBoolean()) {
        //     addCommands(alignToReefCommand);
        //     return;
        // }

        Command followPathCommand = AutoBuilder.followPath(
                Pathfinding.generateReefPath(swerve.getPose(), side, relativePos, swerve.getFieldSpeeds()));

        addCommands(followPathCommand.handleInterrupt(() -> alignToReefCommand.end(true)), alignToReefCommand);
    }

    public Distance getDistanceToTarget() {
        return alignToReefCommand.getDistanceToTarget();
    }

    public Trigger withinRangeTrigger(Distance range) {
        return alignToReefCommand.withinDistanceToTarget(range);
    }
}
