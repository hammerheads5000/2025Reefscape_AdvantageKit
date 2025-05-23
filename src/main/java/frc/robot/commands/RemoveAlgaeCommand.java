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

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class RemoveAlgaeCommand extends SequentialCommandGroup {
    /** Creates a new RemoveAlgaeCommand. */
    public RemoveAlgaeCommand(int side, Swerve swerve, Elevator elevator, AlgaeManipulator algaeManipulator) {
        // Add your commands in the addCommands() call, e.g.
        // addCommands(new FooCommand(), new BarCommand());

        Command pathfindCommand = AutoBuilder.followPath(
                Pathfinding.generateReefPath(swerve.getPose(), side, 0, swerve.getFieldSpeeds()));

        Pose2d farPose = AlignToReefCommands.getReefPose(side, 0)
                .transformBy(new Transform2d(
                        new Translation2d(PathConstants.ALGAE_DEPLOY_DISTANCE.unaryMinus(), Meters.zero()),
                        new Rotation2d()));

        AlignToPoseCommand approachAlignCommand = new AlignToPoseCommand(
                farPose, AlignConstants.ALGAE_PID_TRANSLATION, AlignConstants.ALGAE_PID_ANGLE, swerve);

        AlignToPoseCommand pickAlignCommand = new AlignToPoseCommand(
                AlignToReefCommands.getReefPose(side, 0),
                AlignConstants.ALGAE_PID_TRANSLATION,
                AlignConstants.ALGAE_PID_ANGLE,
                swerve);

        AlignToPoseCommand pullAlignCommand = new AlignToPoseCommand(
                farPose, AlignConstants.ALGAE_PID_TRANSLATION, AlignConstants.ALGAE_PID_ANGLE, swerve);

        addCommands(
                pathfindCommand.until(approachAlignCommand.withinDistanceToTarget(PathConstants.ALGAE_DEPLOY_DISTANCE)),
                approachAlignCommand.alongWith(elevator.goToAlgaeCommand(side, false)),
                algaeManipulator.intakeCommand().deadlineFor(pickAlignCommand),
                pullAlignCommand.alongWith(Commands.waitUntil(pickAlignCommand
                                .withinDistanceToTarget(PathConstants.PULL_DISTANCE)
                                .negate())
                        .andThen(elevator.goToIntakePosCommand(true))));
    }

    public RemoveAlgaeCommand(String token, Swerve swerve, Elevator elevator, AlgaeManipulator algaeManipulator) {
        this(
                FieldConstants.LETTER_TO_SIDE_AND_RELATIVE.get(token.charAt(0)).getFirst(),
                swerve,
                elevator,
                algaeManipulator);
    }
}
