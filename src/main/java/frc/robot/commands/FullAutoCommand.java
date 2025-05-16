// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.Pair;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.NTConstants;
import frc.robot.Constants.PathConstants;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.endeffector.EndEffector;
import frc.robot.subsystems.swerve.Swerve;
import java.util.Set;
import java.util.function.Supplier;

/** Command that executes an auto built from a descriptor string */
public class FullAutoCommand extends SequentialCommandGroup {
    Swerve swerve;
    Elevator elevator;
    EndEffector endEffector;

    private Command getStationCommand(int station, int relativePos) {
        Command command = ApproachCoralStationCommands.pathfindCommand(station, relativePos, swerve);

        if (Constants.CURRENT_MODE == Constants.SIM_MODE) {
            command = command.alongWith(elevator.goToIntakePosCommand(false))
                    .andThen(new ScheduleCommand(endEffector.coolerIntakeCommand()));
        } else {
            command = command.alongWith(elevator.goToIntakePosCommand(false))
                    .alongWith(new ScheduleCommand(endEffector
                            .coolerIntakeCommand() // use ScheduleCommand to branch
                            // off
                            .beforeStarting(Commands.waitTime(PathConstants.INTAKE_WAIT_TIME)))) // wait slightly to
                    // start intake to
                    // avoid stopping early
                    .until(endEffector.coralDetectedTrigger)
                    .andThen(Commands.waitUntil(endEffector.coralDetectedTrigger));
        }
        return command;
    }

    private Command getStationCommand(int station) {
        return getStationCommand(station, 0);
    }

    private Command getElevatorPosCommand(char level) {
        switch (level) {
            case '1':
                return elevator.goToL1Command(false);
            case '2':
                return elevator.goToL2Command(false);
            case '3':
                return elevator.goToL3Command(false);
            case '4':
                return elevator.goToL4Command(false);
            default:
                System.err.println("ERROR: Invalid auto level token " + level);
                return elevator.goToL4Command(false);
        }
    }

    private Command getElevatorTrackCommand(char level, Supplier<Distance> distanceToReef) {
        switch (level) {
            case '1':
                return elevator.trackL1Command(distanceToReef);
            case '2':
                return elevator.trackL2Command(distanceToReef);
            case '3':
                return elevator.trackL3Command(distanceToReef);
            case '4':
                return elevator.trackL4Command(distanceToReef);
            default:
                System.err.println("ERROR: Invalid auto level token " + level);
                return elevator.trackL4Command(distanceToReef);
        }
    }

    private Command getElevatorPosCommand() {
        char level = NTConstants.REEF_TELEOP_AUTO_ENTRY.get().charAt(1);
        return getElevatorPosCommand(level);
    }

    private Command getElevatorTrackCommand(Supplier<Distance> distanceToReef) {
        char level = NTConstants.REEF_TELEOP_AUTO_ENTRY.get().charAt(1);
        return getElevatorTrackCommand(level, distanceToReef);
    }

    private Command getReefCommand(int side, double relativePos, char level) {
        Command commandToAdd;
        Command elevatorPosCommand;
        if (DriverStation.isAutonomous()) {
            elevatorPosCommand = getElevatorPosCommand(level);
        } else {
            elevatorPosCommand = Commands.defer(this::getElevatorPosCommand, Set.of(elevator));
        }
        Command endEffectorCommand = endEffector.scoreCommand();
        if (level == '1') {
            endEffectorCommand = relativePos == 1 ? endEffector.troughLeftCommand() : endEffector.troughRightCommand();

            relativePos *= PathConstants.L1_RELATIVE_POS;
        }

        ApproachReefCommand approachReefCommand = new ApproachReefCommand(side, relativePos, swerve);

        Distance deployDistance;
        switch (Elevator.levelToStage(level)) {
            case STAGE1:
                deployDistance = PathConstants.STAGE1_DEPLOY_DISTANCE;
                break;
            case STAGE2:
                deployDistance = PathConstants.STAGE2_DEPLOY_DISTANCE;
                break;
            case STAGE3:
                deployDistance = PathConstants.STAGE3_DEPLOY_DISTANCE;
                break;
            default:
                deployDistance = PathConstants.STAGE1_DEPLOY_DISTANCE;
        }

        commandToAdd = approachReefCommand
                .alongWith(Commands.waitUntil(approachReefCommand.withinRangeTrigger(deployDistance))
                        .andThen(Commands.waitUntil(endEffector.hasCoralTrigger))
                        .andThen(elevatorPosCommand))
                .andThen(endEffectorCommand.asProxy())
                .andThen(Commands.waitTime(PathConstants.AFTER_WAIT_TIME))
                .andThen(elevator.goToIntakePosCommand(true));

        return commandToAdd;
    }

    private Command getTrackedReefCommand(int side, double relativePos, char level) {
        Command commandToAdd;
        Command elevatorPosCommand;

        ApproachReefCommand approachReefCommand = new ApproachReefCommand(side, relativePos, swerve);

        if (DriverStation.isAutonomous()) {
            elevatorPosCommand = getElevatorTrackCommand(level, approachReefCommand::getDistanceToTarget);
        } else {
            elevatorPosCommand = Commands.defer(
                    () -> this.getElevatorTrackCommand(approachReefCommand::getDistanceToTarget), Set.of(elevator));
        }
        Command endEffectorCommand = endEffector.scoreCommand();
        if (level == '1') {
            endEffectorCommand = relativePos == 1 ? endEffector.troughLeftCommand() : endEffector.troughRightCommand();

            relativePos *= PathConstants.L1_RELATIVE_POS;
        }

        Distance deployDistance;
        switch (Elevator.levelToStage(level)) {
            case STAGE1:
                deployDistance = PathConstants.STAGE1_DEPLOY_DISTANCE;
                break;
            case STAGE2:
                deployDistance = PathConstants.STAGE2_DEPLOY_DISTANCE;
                break;
            case STAGE3:
                deployDistance = PathConstants.STAGE3_DEPLOY_DISTANCE;
                break;
            default:
                deployDistance = PathConstants.STAGE1_DEPLOY_DISTANCE;
        }

        commandToAdd = approachReefCommand
                .alongWith(Commands.waitUntil(approachReefCommand.withinRangeTrigger(deployDistance))
                        .andThen(Commands.waitUntil(endEffector.hasCoralTrigger))
                        .andThen(elevatorPosCommand))
                .until(approachReefCommand.withinRangeTrigger(ElevatorConstants.MAX_SHOOT_DISTANCE))
                .andThen(endEffectorCommand.asProxy())
                .andThen(Commands.waitTime(PathConstants.AFTER_WAIT_TIME))
                .andThen(elevator.goToIntakePosCommand(true));

        return commandToAdd;
    }

    private Command commandFromToken(String token) {
        Command commandToAdd;

        if (token.charAt(0) == 'S') {
            int station = token.charAt(1) == '0' ? 0 : 1;

            Command stationCommand;

            if (token.length() == 3) {
                int relativePos;
                switch (token.charAt(2)) {
                    case 'L':
                        relativePos = 1;
                        break;
                    case 'C':
                        relativePos = 0;
                        break;
                    case 'R':
                        relativePos = -1;
                        break;
                    default:
                        relativePos = 0;
                        System.err.println("ERROR: Invalid auto station token: " + token);
                        break;
                }

                stationCommand = getStationCommand(station, relativePos);
            } else {
                stationCommand = getStationCommand(station);
            }

            commandToAdd = Commands.defer(() -> stationCommand, Set.of(swerve, elevator));
        } else {
            Pair<Integer, Integer> sidePosPair;
            if (!FieldConstants.LETTER_TO_SIDE_AND_RELATIVE.containsKey(token.charAt(0))) {
                System.err.println("ERROR: Invalid auto branch token " + token);
                return Commands.none();
            }
            sidePosPair = FieldConstants.LETTER_TO_SIDE_AND_RELATIVE.get(token.charAt(0));

            int side = sidePosPair.getFirst();
            int relativePos = sidePosPair.getSecond();

            commandToAdd =
                    Commands.defer(() -> getReefCommand(side, relativePos, token.charAt(1)), Set.of(swerve, elevator));
        }

        return commandToAdd;
    }

    /**
     * Create FullAutoCommand
     *
     * @param descriptorString S0-1[L,C,R] (station and optional relative position), A-L1-4 (branch and level), space
     *     separated (e.g. "E4 S0C A3 S1 K1")
     * @param swerve
     */
    public FullAutoCommand(String descriptorString, Swerve swerve, Elevator elevator, EndEffector endEffector) {
        this.swerve = swerve;
        this.elevator = elevator;
        this.endEffector = endEffector;

        String[] tokens = descriptorString.split(" ");

        for (String token : tokens) {
            if (token.length() < 2) {
                System.err.println("ERROR: Invalid auto token length " + token);
                continue;
            }
            addCommands(Commands.defer(() -> commandFromToken(token), Set.of(swerve, elevator)));
        }
    }
}
