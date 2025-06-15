// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import static frc.robot.Constants.FieldConstants.branchPositions;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.FieldConstants.ReefLevel;
import frc.robot.commands.AlignToReefCommands;
import frc.robot.commands.FullAutoCommand;
import frc.robot.commands.RemoveAlgaeCommand;
import frc.robot.commands.ScoreInBargeCommand;
import frc.robot.subsystems.algaemanipulator.AlgaeManipulator;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.endeffector.EndEffector;
import frc.robot.subsystems.swerve.Swerve;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Map;
import java.util.Set;
import org.littletonrobotics.junction.Logger;

/** Add your docs here. */
public class Strategy {
    static final Translation2d[] reef = new Translation2d[12];
    static final double SPEED = 1.5;
    static final double SCORE_TIME = 1.6;
    static final double STATION_TIME = 0.5;
    static final double ALGAE_TIME = 2;
    static final double BARGE_TIME = 2.5;

    private static Map<Integer, Integer> CWToCCW = Map.of(
            0, 0,
            1, 5,
            2, 4,
            3, 3,
            4, 2,
            5, 1);

    static {
        int i = 0;
        while (i < 12) {
            reef[i] = AlignToReefCommands.getReefPose(CWToCCW.get(i / 2), 1, false)
                    .getTranslation();
            i++;
            reef[i] = AlignToReefCommands.getReefPose(CWToCCW.get(i / 2), -1, false)
                    .getTranslation();
            i++;
        }
    }

    static Translation2d STATION0 = FieldConstants.STATION_0.getTranslation();
    static Translation2d STATION1 = FieldConstants.STATION_1.getTranslation();
    static Translation2d BARGE = FieldConstants.BARGE_POSES.get('G').getTranslation();

    private static record RobotState(Translation2d pos, boolean hasCoral, boolean hasAlgae, String name) {
        @Override
        public final String toString() {
            return name + "(" + (hasCoral ? "C " : "  ") + (hasAlgae ? "A)" : " )");
        }
    }

    public static record ReefState(boolean[][] coral, boolean[] algae) {
        public ReefState() {
            this(new boolean[12][4], new boolean[6]);
        }

        public ReefState(boolean allCoral, boolean allAlgae) {
            this();
            for (int i = 0; i < coral.length; i++) {
                for (int j = 0; j < coral[i].length; j++) {
                    coral[i][j] = allCoral;
                }
            }
            for (int i = 0; i < algae.length; i++) {
                algae[i] = allAlgae;
            }
        }

        @Override
        public final String toString() {
            String str = "Coral: ";
            for (boolean[] branch : coral) {
                str += Arrays.toString(branch) + "\n";
            }
            str += "\nAlgae: " + algae;
            return str;
        }

        @Override
        public final boolean equals(Object other) {
            if (this == other) {
                return true;
            }
            if (!(other instanceof ReefState)) {
                return false;
            }
            ReefState otherReef = (ReefState) other;
            for (int i = 0; i < coral.length; i++) {
                if (!Arrays.equals(coral[i], otherReef.coral[i])) {
                    return false;
                }
            }
            return true;
        }
    }

    static final ReefState FULL_REEF = new ReefState(true, false);

    static ReefState addedCoral(ReefState reefState, int branch, int level) {
        boolean[][] coral = new boolean[12][4];
        for (int i = 0; i < coral.length; i++) {
            coral[i] = reefState.coral[i].clone();
        }
        coral[branch][level - 1] = true;
        return new ReefState(coral, reefState.algae);
    }

    static ReefState removedAlgae(ReefState reefState, int side) {
        boolean[] algae = reefState.algae.clone();
        algae[side] = false;
        return new ReefState(reefState.coral, algae);
    }

    private static record Move(RobotState from, RobotState to, ReefState reefState, int points, double time) {
        @Override
        public final String toString() {
            return from + " -> " + to + ", " + points + "pts, " + Math.round(time * 100) / 100.0 + "s";
            // return "Move(" + from + " -> " + to + ", " + reefState + ", " + points +
            // "pts, "
            // + Math.round(time * 100) / 100.0 + "s)";
        }
    }

    static Map<Integer, Integer> pointsMap = Map.of(
            1, 2,
            2, 3,
            3, 4,
            4, 5);

    static double getPtsPerSec(Move move) {
        return move.points / move.time;
    }

    static Move scoreCoral(RobotState from, int branch, int level, ReefState reefState) {
        RobotState to = new RobotState(reef[branch], false, false, "Cor " + (char) ('A' + branch) + "" + level + " ");
        ReefState newReefState = addedCoral(reefState, branch, level);
        double dist = from.pos.getDistance(to.pos);
        double time = dist / SPEED + SCORE_TIME;
        int points = pointsMap.get(level);

        return new Move(from, to, newReefState, points, time);
    }

    static boolean canScoreCoral(int branch, int level, ReefState reefState) {
        return (!reefState.coral[branch][level - 1]
                && !((level == 3 || (level == 2 && (branch - 1) % 4 >= 2)) && reefState.algae[branch / 2]));
    }

    static Move scoreAlgae(RobotState from, boolean hasCoral, ReefState reefState) {
        RobotState to = new RobotState(BARGE, hasCoral, false, "Barge  ");
        double dist = from.pos.getDistance(to.pos);
        double time = dist / SPEED + ALGAE_TIME;
        int points = 4;
        return new Move(from, to, reefState, points, time);
    }

    static Move pickupAlgae(RobotState from, int side, boolean hasCoral, ReefState reefState) {
        RobotState to = new RobotState(
                AlignToReefCommands.getReefPose(CWToCCW.get(side), 0, false).getTranslation(),
                hasCoral,
                true,
                "Algae " + side + "");
        ReefState newReefState = removedAlgae(reefState, side);
        double dist = from.pos.getDistance(to.pos);
        double time = dist / SPEED + STATION_TIME;
        int points = 0;
        return new Move(from, to, newReefState, points, time);
    }

    static Move pickupCoral(RobotState from, int station, boolean hasAlgae, ReefState reefState) {
        RobotState to = new RobotState(
                station == 1 ? STATION1 : STATION0, true, hasAlgae, "S" + (station == 1 ? "1     " : "0     "));
        double dist = from.pos.getDistance(to.pos);
        double time = dist / SPEED + STATION_TIME;
        int points = 0;
        return new Move(from, to, reefState, points, time);
    }

    static ArrayList<Move> possibleMoves(RobotState state, ReefState reefState) {
        ArrayList<Move> moves = new ArrayList<>();
        if (!state.hasCoral) {
            moves.add(pickupCoral(state, 0, state.hasAlgae, reefState));
            moves.add(pickupCoral(state, 1, state.hasAlgae, reefState));
        }

        if (state.hasAlgae) {
            moves.add(scoreAlgae(state, state.hasCoral, reefState));
        } else {
            if (state.hasCoral) {
                for (int branch = 0; branch < 12; branch++) {
                    for (int level = 4; level > 0; level--) {
                        if (canScoreCoral(branch, level, reefState)) {
                            moves.add(scoreCoral(state, branch, level, reefState));
                            break;
                        }
                    }
                }
            }

            for (int side = 0; side < 6; side++) {
                if (reefState.algae[side]) {
                    moves.add(pickupAlgae(state, side, state.hasCoral, reefState));
                }
            }
        }

        return moves;
    }

    static Move bestImmediateMove(RobotState state, ReefState reefState) {
        ArrayList<Move> moves = possibleMoves(state, reefState);
        if (moves.isEmpty()) {
            return new Move(state, state, reefState, 0, 1);
        }
        Move bestMove = moves.get(0);
        for (Move move : moves) {
            if (getPtsPerSec(move) > getPtsPerSec(bestMove)) {
                bestMove = move;
            }
        }
        return bestMove;
    }

    private static record BestMoveData(ArrayList<Move> moves, double bestPoints, double bestTime) {}

    static BestMoveData getBestMoves(RobotState state, int depth, ReefState reefState) {
        if (depth == 1) {
            ArrayList<Move> bestMoves = new ArrayList<>();
            bestMoves.add(bestImmediateMove(state, reefState));
            return new BestMoveData(bestMoves, bestMoves.get(0).points, bestMoves.get(0).time);
        }

        ArrayList<Move> possMoves = possibleMoves(state, reefState);
        if (possMoves.isEmpty()) {
            return new BestMoveData(new ArrayList<>(Arrays.asList(new Move(state, state, reefState, 0, 1))), 0, 1);
        }

        BestMoveData bestMoveData = getBestMoves(possMoves.get(0).to, depth - 1, possMoves.get(0).reefState);
        bestMoveData.moves.add(0, possMoves.get(0));
        bestMoveData = new BestMoveData(
                bestMoveData.moves,
                bestMoveData.bestPoints + possMoves.get(0).points,
                bestMoveData.bestTime + possMoves.get(0).time);
        for (Move move : possMoves) {
            BestMoveData nextMoveData = getBestMoves(move.to, depth - 1, move.reefState);
            nextMoveData.moves.add(0, move);
            nextMoveData = new BestMoveData(
                    nextMoveData.moves, nextMoveData.bestPoints + move.points, nextMoveData.bestTime + move.time);

            if (nextMoveData.bestPoints / nextMoveData.bestTime > bestMoveData.bestPoints / bestMoveData.bestTime) {
                bestMoveData = nextMoveData;
            } else if (nextMoveData.bestPoints / nextMoveData.bestTime
                            == bestMoveData.bestPoints / bestMoveData.bestTime
                    && nextMoveData.bestPoints > bestMoveData.bestPoints) {
                bestMoveData = nextMoveData;
            }
        }

        return bestMoveData;
    }

    static BestMoveData bestMoveSequence(RobotState state, ReefState reefState, int lookahead, int length) {
        ArrayList<Move> bestMoves = new ArrayList<>();
        double bestPoints = 0;
        double bestTime = 0;
        if (length == -1) {
            while (!reefState.equals(FULL_REEF)) {
                BestMoveData nextBest = getBestMoves(state, lookahead, reefState);
                bestMoves.addAll(nextBest.moves);
                bestPoints += nextBest.bestPoints;
                bestTime += nextBest.bestTime;
                reefState = nextBest.moves.get(nextBest.moves.size() - 1).reefState;
                state = nextBest.moves.get(nextBest.moves.size() - 1).to;
            }
        } else {
            for (int i = 0; i < length; i++) {
                BestMoveData nextBest = getBestMoves(state, lookahead, reefState);
                bestMoves.addAll(nextBest.moves);
                bestPoints += nextBest.bestPoints;
                bestTime += nextBest.bestTime;
                reefState = nextBest.moves.get(nextBest.moves.size() - 1).reefState;
                state = nextBest.moves.get(nextBest.moves.size() - 1).to;
            }
        }

        return new BestMoveData(bestMoves, bestPoints, bestTime);
    }

    public static void test() {
        ReefState currentReef = new ReefState(false, true);
        // System.out.println(possibleMoves(new RobotState(BARGE, true, false, "Barge
        // "), currentReef));

        BestMoveData bestMoveData =
                bestMoveSequence(new RobotState(BARGE, false, false, "Start  "), currentReef, 4, -1);
        // System.out.println(bestMoveData.moves.get(bestMoveData.moves.size() - 1));
        bestMoveData.moves.forEach(m -> System.out.println(m));
        System.out.println(bestMoveData.bestPoints + " pts in " + Math.round(bestMoveData.bestTime * 100) / 100.0
                + " seconds: " + Math.round(bestMoveData.bestPoints / bestMoveData.bestTime * 100) / 100.0 + "pts/s");
    }

    public static Command doBestMove(
            Swerve swerve,
            Elevator elevator,
            EndEffector endEffector,
            AlgaeManipulator algaeManipulator,
            int lookahead) {
        return doBestMove(swerve, elevator, endEffector, algaeManipulator, lookahead, new ReefState(false, true));
    }

    public static Command doBestMove(
            Swerve swerve,
            Elevator elevator,
            EndEffector endEffector,
            AlgaeManipulator algaeManipulator,
            int lookahead,
            ReefState reefState) {
        RobotState startState = new RobotState(
                swerve.getPose().getTranslation(),
                endEffector.hasCoralTrigger.getAsBoolean(),
                algaeManipulator.algaeDetectedTrigger.getAsBoolean(),
                "Start  ");
        Move bestMove = getBestMoves(startState, lookahead, reefState).moves.get(0);
        ReefState nextReef = bestMove.reefState;
        System.out.println(nextReef);
        String name = bestMove.to.name;
        Command cmd;
        switch (name.split(" ")[0]) {
            case "Cor":
                cmd = new FullAutoCommand(name.split(" ")[1], swerve, elevator, endEffector, algaeManipulator);
                break;
            case "Algae":
                cmd = new RemoveAlgaeCommand(CWToCCW.get(name.charAt(6) - '0'), swerve, elevator, algaeManipulator);
                break;
            case "Barge":
                cmd = new ScoreInBargeCommand('G', swerve, elevator, algaeManipulator);
                break;
            default:
                cmd = new FullAutoCommand(name.split(" ")[0], swerve, elevator, endEffector, algaeManipulator);
                break;
        }
        return cmd.finallyDo(() -> updateReef(nextReef))
                .andThen(Commands.defer(
                        () -> doBestMove(swerve, elevator, endEffector, algaeManipulator, lookahead, nextReef),
                        Set.of(swerve, elevator)));
    }

    public static void updateReef(ReefState reefState) {
        ArrayList<Pose3d> corals = new ArrayList<>();
        for (int i = 0; i < reefState.coral.length; i++) {
            for (int j = 0; j < reefState.coral[i].length; j++) {
                if (reefState.coral[i][j]) {
                    corals.add(branchPositions.get(i).get(ReefLevel.fromLevel(j)));
                }
            }
        }
        Logger.recordOutput("Mechanism3d/Reef/Coral", corals.toArray(Pose3d[]::new));
        ArrayList<Translation3d> algae = new ArrayList<>();
        for (int i = 0; i < 6; i++) {
            if (reefState.algae[i]) {
                Translation3d branch1 = FieldConstants.branchPositions
                        .get(i * 2)
                        .get(ReefLevel.L2)
                        .getTranslation();
                Translation3d branch2 = FieldConstants.branchPositions
                        .get(i * 2 + 1)
                        .get(ReefLevel.L3)
                        .getTranslation();
                algae.add(branch1.interpolate(branch2, 0.5)
                        .plus(new Translation3d(
                                0.0,
                                0.0,
                                ((i % 2 == 0) ? branch2.getZ() - branch1.getZ() : 0.0) + Units.inchesToMeters(-0.7))));
            }
        }
        Logger.recordOutput("Mechanism3d/Reef/Algae", algae.toArray(Translation3d[]::new));
    }
}
