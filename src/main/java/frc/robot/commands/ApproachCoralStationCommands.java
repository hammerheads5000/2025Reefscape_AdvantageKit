// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static edu.wpi.first.units.Units.Meters;
import static frc.robot.Constants.FieldConstants.*;
import static frc.robot.Constants.INST;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.PathConstants;
import frc.robot.subsystems.swerve.Swerve;

/** Container class for approaching/moving to coral station */
public class ApproachCoralStationCommands {
    /**
     * @param station 0 for right station, 1 for left from operator perspective
     * @param relativePos -1 for right, 0 for center, 1 for left from operator perspective
     * @return Station
     */
    public static Pose2d getStationPose(int station, int relativePos) {
        Pose2d pose = station == 1 ? STATION_1 : STATION_0;
        Translation2d translation = new Translation2d(
                PathConstants.STATION_APPROACH_DISTANCE, PathConstants.SIDE_STATION_OFFSET.times(relativePos));
        pose = pose.transformBy(new Transform2d(translation, Rotation2d.kZero));
        if (DriverStation.getAlliance().isPresent()
                && DriverStation.getAlliance().get() == Alliance.Red) {
            pose = AlignToReefCommands.flipPose(pose);
        }
        return pose;
    }

    /** Publishes station poses to NetworkTables */
    public static void testStationPoses() {
        Pose2d[] poses = new Pose2d[] {
            getStationPose(0, -1),
            getStationPose(0, 0),
            getStationPose(0, 1),
            getStationPose(1, -1),
            getStationPose(1, 0),
            getStationPose(1, 1)
        };
        INST.getStructArrayTopic("Station Poses", Pose2d.struct).publish().set(poses);
    }

    /**
     * Generate PathPlannerPath to the desired station, moving around the reef if necessary
     *
     * @param station 0 for right station, 1 for left from operator perspective
     * @param relativePos -1 for right, 0 for center, 1 for left from operator perspective
     * @param swerve
     * @return
     */
    public static Command pathfindCommand(int station, int relativePos, Swerve swerve) {
        if (swerve.getPose()
                        .getTranslation()
                        .getDistance(getStationPose(station, relativePos).getTranslation())
                < PathConstants.MIN_PATH_DISTANCE.in(Meters)) {
            return Commands.none();
        }

        PathPlannerPath path =
                Pathfinding.generateStationPath(swerve.getPose(), station, relativePos, swerve.getFieldSpeeds());

        return AutoBuilder.followPath(path).withName("Station " + station + " Path");
    }
}
