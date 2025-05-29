// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climber;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ClimberConstants;
import org.littletonrobotics.junction.Logger;

public class Climber extends SubsystemBase {
    private final ClimberIO io;
    private final ClimberIOInputsAutoLogged inputs = new ClimberIOInputsAutoLogged();

    private final Debouncer atMaxHeight = new Debouncer(0.2, DebounceType.kFalling);

    private Trigger cageDetectedTrigger = new Trigger(() -> inputs.cageDetected).debounce(0.2);

    public Climber(ClimberIO io) {
        this.io = io;

        // cageDetectedTrigger.onTrue(autoClimbCommand());

        SmartDashboard.putData("Climb", climbCommand());
        SmartDashboard.putData("Unclimb", reverseCommand());
        SmartDashboard.putData("Climber Grab", grabCommand());
        SmartDashboard.putData("Climber Ungrab", releaseCommand());
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Climber", inputs);
    }

    public Command climbCommand() {
        return this.startEnd(() -> io.setClimberOutput(ClimberConstants.CLIMB_SPEED), io::stopClimb)
                .withName("Climb");
    }

    public Command reverseCommand() {
        return this.startEnd(() -> io.setClimberOutput(ClimberConstants.RELEASE_SPEED), io::stopClimb)
                .withName("Unclimb");
    }

    public Command grabCommand() {
        return this.startEnd(() -> io.setGrabOutput(ClimberConstants.GRAB_SPEED), io::stopGrab)
                .withName("Grab Cage");
    }

    public Command releaseCommand() {
        return this.startEnd(() -> io.setGrabOutput(ClimberConstants.RELEASE_SPEED), io::stopGrab)
                .withName("Release Cage");
    }

    public Command goToGrabPosCommand() {
        return reverseCommand()
                .until(() -> inputs.pos.lte(ClimberConstants.GRAB_ANGLE))
                .finallyDo(() -> io.setGrabOutput(ClimberConstants.GRAB_SPEED))
                .withName("Go to Cage Grab Pos");
    }

    public Command autoClimbCommand() {
        return Commands.repeatingSequence(
                climbCommand().until(() -> inputs.pos.gte(ClimberConstants.MAX_CLIMB_ANGLE)),
                Commands.waitUntil(() -> !atMaxHeight.calculate(inputs.pos.gte(ClimberConstants.MAX_CLIMB_ANGLE))))
                .withName("Auto Climb");
    }
}
