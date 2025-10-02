// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static edu.wpi.first.units.Units.Meters;
import static frc.robot.Constants.INST;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.FlippingUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.PathConstants;
import frc.robot.subsystems.swerve.Swerve;

/** Container class for approaching/moving to barge */
public class ApproachBargeCommands {
    public static Pose2d getBargePose(char pos, boolean isRed) {
        Pose2d pose = FieldConstants.BARGE_POSES.get(pos);

        if (isRed) {
            pose = FlippingUtil.flipFieldPose(pose);
        }
        return pose;
    }

    /**
     * Get given barge pose, flipped based on alliance
     *
     * @param pos 1,2,3 for barge positions from left to right
     * @return Barge pose
     */
    public static Pose2d getBargePose(char pos) {
        return getBargePose(pos, DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red);
    }

    /** Publishes barge poses to NetworkTables */
    public static void testBargePoses() {
        Pose2d[] bluePoses =
                new Pose2d[] {getBargePose('1', false), getBargePose('2', false), getBargePose('3', false)};

        Pose2d[] redPoses = new Pose2d[] {getBargePose('1', true), getBargePose('2', true), getBargePose('3', true)};

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
     * @param pos 1,2,3 for barge positions from left to right
     * @param swerve
     * @return AutoBuilder follow path command
     */
    public static Command pathfindCommand(char pos, Swerve swerve) {
        if (swerve.getPose().getTranslation().getDistance(getBargePose(pos).getTranslation())
                < PathConstants.MIN_PATH_DISTANCE.in(Meters)) {
            return Commands.none();
        }

        PathPlannerPath path = Pathfinding.generateBargePath(swerve.getPose(), pos, swerve.getFieldSpeeds());

        return AutoBuilder.followPath(path).withName("Barge Path " + pos);
    }
}
