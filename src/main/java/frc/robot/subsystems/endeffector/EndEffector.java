// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.endeffector;

import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.Constants.EndEffectorConstants;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class EndEffector extends SubsystemBase {
    private final EndEffectorIO io;
    private final EndEffectorIOInputsAutoLogged inputs = new EndEffectorIOInputsAutoLogged();

    private Trigger isSim = new Trigger(() -> Constants.CURRENT_MODE == Constants.SIM_MODE);
    private boolean simHasCoral = true;
    public Trigger coralDetectedTrigger = new Trigger(() -> inputs.frontLidar || inputs.backLidar || inputs.intakeLidar)
            .debounce(0.1)
            .or(isSim);

    @AutoLogOutput
    public Trigger hasCoralTrigger = new Trigger(() -> inputs.frontLidar && !inputs.backLidar)
            .debounce(0.1)
            .or(isSim.and(() -> simHasCoral));

    public Trigger coralReleasedTrigger = new Trigger(() -> !inputs.frontLidar && !inputs.backLidar)
            .debounce(0.1)
            .or(isSim);
    public Trigger stalledTrigger = new Trigger(this::isStalled)
            .debounce(EndEffectorConstants.STALL_TIME.in(Seconds))
            .and(isSim.negate());

    private final Alert leftDisconnectedAlert = new Alert("Disconnected left end effector motor.", AlertType.kError);
    private final Alert rightDisconnectedAlert = new Alert("Disconnected right end effector motor.", AlertType.kError);

    public EndEffector(EndEffectorIO io) {
        this.io = io;

        SmartDashboard.putData("End Effector Intake", coolerIntakeCommand());
        SmartDashboard.putData("End Effector Shoot", scoreCommand());
        SmartDashboard.putData("End Effector Trough Left", troughLeftCommand());
        SmartDashboard.putData("End Effector Trough Right", troughRightCommand());
        SmartDashboard.putData("End Effector Reverse", runCommand(EndEffectorConstants.INTAKE_SPEED.unaryMinus()));
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("EndEffector", inputs);

        leftDisconnectedAlert.set(!inputs.leftConnected);
        rightDisconnectedAlert.set(!inputs.rightConnected);
    }

    private boolean isStalled() {
        return inputs.leftVelocity.magnitude() <= EndEffectorConstants.MIN_VEL.magnitude()
                || inputs.rightVelocity.magnitude() <= EndEffectorConstants.MIN_VEL.magnitude();
    }

    private Command runCommand(Voltage left, Voltage right) {
        return this.startEnd(() -> io.setSpeeds(left, right), io::stop);
    }

    public Command runCommand(Voltage speed) {
        return runCommand(speed, speed);
    }

    public Command intakeCommand() {
        if (isSim.getAsBoolean()) {
            return this.runOnce(() -> simHasCoral = true);
        }
        return runCommand(EndEffectorConstants.INTAKE_SPEED).until(hasCoralTrigger);
    }

    public Command coolerIntakeCommand() {
        if (isSim.getAsBoolean()) {
            return this.runOnce(() -> simHasCoral = true);
        }
        return Commands.repeatingSequence(
                        runCommand(EndEffectorConstants.INTAKE_SPEED).until(stalledTrigger),
                        Commands.waitTime(EndEffectorConstants.COOLER_INTAKE_CYCLE))
                .until(hasCoralTrigger);
    }

    public Command scoreCommand() {
        if (isSim.getAsBoolean()) {
            return this.runOnce(() -> simHasCoral = false);
        }
        return runCommand(EndEffectorConstants.SCORE_SPEED).until(coralReleasedTrigger);
    }

    public Command troughLeftCommand() {
        if (isSim.getAsBoolean()) {
            return this.runOnce(() -> simHasCoral = false);
        }
        return runCommand(EndEffectorConstants.SLOW_TROUGH_SPEED, EndEffectorConstants.FAST_TROUGH_SPEED)
                .until(coralReleasedTrigger);
    }

    public Command troughRightCommand() {
        if (isSim.getAsBoolean()) {
            return this.runOnce(() -> simHasCoral = false);
        }
        return runCommand(EndEffectorConstants.FAST_TROUGH_SPEED, EndEffectorConstants.SLOW_TROUGH_SPEED)
                .until(coralReleasedTrigger);
    }
}
