// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static edu.wpi.first.units.Units.Meters;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.AlignConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.PathConstants;
import frc.robot.subsystems.algaemanipulator.AlgaeManipulator;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.swerve.Swerve;

/** Command to remove algae from the reef by following a path, aligning to the reef, and removing the algae. */
public class RemoveAlgaeCommand extends SequentialCommandGroup {
    /**
     * Creates a new RemoveAlgaeCommand.
     *
     * @param side The side of the reef (0-5 starting at A/B going clockwise)
     */
    public RemoveAlgaeCommand(int side, Swerve swerve, Elevator elevator, AlgaeManipulator algaeManipulator) {
        Command pathfindCommand = AutoBuilder.followPath(
                Pathfinding.generateReefPath(swerve.getPose(), side, 0, swerve.getFieldSpeeds()));

        // offset from reef
        Pose2d farPose = AlignToReefCommands.getReefPose(side, 0)
                .transformBy(new Transform2d(
                        new Translation2d(PathConstants.ALGAE_DEPLOY_DISTANCE.unaryMinus(), Meters.zero()),
                        new Rotation2d()));

        AlignToPoseCommand approachAlignCommand = new AlignToPoseCommand(
                farPose, AlignConstants.ALGAE_PICK_TRANSLATION, AlignConstants.ALGAE_PICK_ANGLE, swerve);

        AlignToPoseCommand pickAlignCommand = new AlignToPoseCommand(
                AlignToReefCommands.getReefPose(side, 0)
                        .transformBy(new Transform2d(
                                new Translation2d(PathConstants.ALGAE_EXTRA_DISTANCE_IN, Meters.zero()),
                                Rotation2d.kZero)),
                AlignConstants.ALGAE_PICK_TRANSLATION,
                AlignConstants.ALGAE_PICK_ANGLE,
                swerve);

        AlignToPoseCommand pullAlignCommand = new AlignToPoseCommand(
                farPose, AlignConstants.ALGAE_PICK_TRANSLATION, AlignConstants.ALGAE_PICK_ANGLE, swerve);

        addCommands(
                pathfindCommand.until(approachAlignCommand.withinDistanceToTarget(PathConstants.ALGAE_DEPLOY_DISTANCE)),
                approachAlignCommand.alongWith(elevator.goToAlgaeCommand(side, false)),
                algaeManipulator.intakeCommand().asProxy().deadlineFor(pickAlignCommand),
                pullAlignCommand.alongWith(Commands.waitUntil(pickAlignCommand
                                .withinDistanceToTarget(PathConstants.PULL_DISTANCE)
                                .negate())
                        .andThen(elevator.goToIntakePosCommand(true))));
    }

    /**
     * Creates a new RemoveAlgaeCommand
     *
     * @param token The token from the reef descriptor to determine side
     */
    public RemoveAlgaeCommand(String token, Swerve swerve, Elevator elevator, AlgaeManipulator algaeManipulator) {
        this(
                FieldConstants.LETTER_TO_SIDE_AND_RELATIVE.get(token.charAt(0)).getFirst(),
                swerve,
                elevator,
                algaeManipulator);
    }
}
