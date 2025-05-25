// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.PathConstants;
import frc.robot.subsystems.algaemanipulator.AlgaeManipulator;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.swerve.Swerve;
import java.util.Set;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ScoreInBargeCommand extends SequentialCommandGroup {
    /** Creates a new ScoreInBargeCommand. */
    public ScoreInBargeCommand(char pos, Swerve swerve, Elevator elevator, AlgaeManipulator algaeManipulator) {
        if (!Set.of('F', 'G', 'H', 'I').contains(pos)) {
            pos = 'I';
            System.err.println("Invalid barge position, defaulting to I");
        }
        addCommands(
                ApproachBargeCommands.pathfindCommand(pos, swerve),
                swerve.runOnce(swerve::stop),
                elevator.goToBargeCommand(false),
                Commands.waitTime(PathConstants.BARGE_SETTLE_TIME),
                algaeManipulator.ejectCommand(),
                elevator.goToIntakePosCommand(true));
    }
}
