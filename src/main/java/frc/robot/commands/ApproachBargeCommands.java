// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static edu.wpi.first.units.Units.Meters;
import static frc.robot.Constants.INST;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.PathConstants;
import frc.robot.FieldConstants;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.util.FlipUtil;

/** Container class for approaching/moving to barge */
public class ApproachBargeCommands {
    /** Publishes barge poses to NetworkTables */
    public static void testBargePoses() {
        Pose2d[] bluePoses = new Pose2d[] {
            FieldConstants.getBargePose('F'),
            FieldConstants.getBargePose('G'),
            FieldConstants.getBargePose('H'),
            FieldConstants.getBargePose('I')
        };

        Pose2d[] redPoses = new Pose2d[] {
            FlipUtil.flip(FieldConstants.getBargePose('F')),
            FlipUtil.flip(FieldConstants.getBargePose('G')),
            FlipUtil.flip(FieldConstants.getBargePose('H')),
            FlipUtil.flip(FieldConstants.getBargePose('I'))
        };

        INST.getStructArrayTopic("Barge Poses/Blue Barge Poses", Pose2d.struct)
                .publish()
                .set(bluePoses);
        INST.getStructArrayTopic("Barge Poses/Red Barge Poses", Pose2d.struct)
                .publish()
                .set(redPoses);
    }

    /**
     * Generate PathPlannerPath to the desired barge pos, moving around the reef if necessary
     *
     * @param pos F,G,H,I for barge positions from right to left
     * @param swerve
     * @return AutoBuilder follow path command
     */
    public static Command pathfindCommand(char pos, Swerve swerve) {
        if (swerve.getPose()
                        .getTranslation()
                        .getDistance(FlipUtil.applyAlliance(FieldConstants.getBargePose(pos))
                                .getTranslation())
                < PathConstants.MIN_PATH_DISTANCE.in(Meters)) {
            return Commands.none();
        }

        PathPlannerPath path = Pathfinding.generateBargePath(swerve.getPose(), pos, swerve.getFieldSpeeds());

        return AutoBuilder.followPath(path).withName("Barge Path " + pos);
    }
}
