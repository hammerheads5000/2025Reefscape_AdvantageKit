// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ClimberConstants;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.swerve.Swerve;

public class ClimbSequence extends SequentialCommandGroup {
    public ClimbSequence(Swerve swerve, Climber climber) {
        addCommands(
                swerve.runOnce(() -> swerve.driveFieldCentric(
                        MetersPerSecond.zero(), MetersPerSecond.zero(), ClimberConstants.SWERVE_TURN_SPEED)),
                Commands.waitTime(ClimberConstants.SWERVE_TURN_TIME),
                swerve.runOnce(swerve::stop),
                climber.autoClimbCommand());
    }
}
