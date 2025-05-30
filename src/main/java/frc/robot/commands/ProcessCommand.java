// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.AlignConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.PathConstants;
import frc.robot.subsystems.algaemanipulator.AlgaeManipulator;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.swerve.Swerve;

/** Pathfind then process algae */
public class ProcessCommand extends SequentialCommandGroup {
    /** Creates a new ProcessCommand. */
    public ProcessCommand(Swerve swerve, Elevator elevator, AlgaeManipulator algaeManipulator) {
        AlignToPoseCommand alignCommand = new AlignToPoseCommand(
                AutoBuilder.shouldFlip()
                        ? AlignToReefCommands.flipPose(FieldConstants.PROCESSOR)
                        : FieldConstants.PROCESSOR,
                AlignConstants.PROCESS_PID_TRANSLATION,
                AlignConstants.PROCESS_PID_ANGLE,
                swerve);

        Command pathfindCommand = Commands.none();

        if (!alignCommand
                .withinDistanceToTarget(PathConstants.MIN_PATH_DISTANCE)
                .getAsBoolean()) {
            PathPlannerPath path = Pathfinding.generateProcessPath(swerve.getPose(), swerve.getFieldSpeeds());
            pathfindCommand = AutoBuilder.followPath(path);
        }

        addCommands(
                Commands.parallel(elevator.goToProcessCommand(false), pathfindCommand.andThen(alignCommand)),
                algaeManipulator.ejectCommand().asProxy());
    }
}
