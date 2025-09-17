// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.endeffector;

import static edu.wpi.first.units.Units.Volts;

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

    @AutoLogOutput
    public Trigger coralDetectedTrigger =
            new Trigger(this::rawHasCoral).debounce(0.1).or(isSim.and(() -> simHasCoral));

    private boolean intaking = false;

    @AutoLogOutput
    public Trigger intakingTrigger = new Trigger(() -> intaking);

    private final Alert leftDisconnectedAlert = new Alert("Disconnected left end effector motor.", AlertType.kError);
    private final Alert rightDisconnectedAlert = new Alert("Disconnected right end effector motor.", AlertType.kError);

    public EndEffector(EndEffectorIO io) {
        this.io = io;

        SmartDashboard.putData("End Effector Intake", intakeCommand());
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

    private boolean rawHasCoral() {
        return inputs.leftTorqueCurrent.gte(EndEffectorConstants.CORAL_DETECTION_CURRENT)
                || inputs.rightTorqueCurrent.gte(EndEffectorConstants.CORAL_DETECTION_CURRENT);
    }

    public void setSpeed(Voltage speed) {
        io.setSpeeds(speed, speed);
    }

    private Command runCommand(Voltage left, Voltage right) {
        return this.startEnd(() -> io.setSpeeds(left, right), io::stop);
    }

    public Command runCommand(Voltage speed) {
        return runCommand(speed, speed).withName("Run End Effector (" + speed.in(Volts) + "V)");
    }

    public Command startIntakeCommand() {
        if (isSim.getAsBoolean()) {
            return this.runOnce(() -> simHasCoral = true).withName("End Effector Intake Start");
        }
        return Commands.runOnce(() -> setSpeed(EndEffectorConstants.INTAKE_SPEED))
                .beforeStarting(() -> intaking = true)
                .withName("End Effector Intake Start");
    }

    public Command stopCommand() {
        return Commands.runOnce(() -> {
                    io.stop();
                    intaking = false;
                })
                .withName("End Effector Stop");
    }

    public Command intakeCommand() {
        if (isSim.getAsBoolean()) {
            return this.runOnce(() -> simHasCoral = true).withName("End Effector Intake");
        }
        return Commands.startEnd(() -> setSpeed(EndEffectorConstants.INTAKE_SPEED), io::stop)
                .beforeStarting(() -> intaking = true)
                .finallyDo(() -> intaking = false)
                .until(coralDetectedTrigger)
                .withName("End Effector Intake");
    }

    public Command scoreCommand() {
        if (isSim.getAsBoolean()) {
            return this.runOnce(() -> simHasCoral = false).withName("End Effector Score");
        }
        return runCommand(EndEffectorConstants.SCORE_SPEED)
                .until(coralDetectedTrigger.negate())
                .withName("End Effector Score");
    }

    public Command troughLeftCommand() {
        if (isSim.getAsBoolean()) {
            return this.runOnce(() -> simHasCoral = false).withName("End Effector Left Trough");
        }
        return runCommand(EndEffectorConstants.SLOW_TROUGH_SPEED, EndEffectorConstants.FAST_TROUGH_SPEED)
                .until(coralDetectedTrigger.negate())
                .withName("End Effector Left Trough");
    }

    public Command troughRightCommand() {
        if (isSim.getAsBoolean()) {
            return this.runOnce(() -> simHasCoral = false).withName("End Effector Trough Right");
        }
        return runCommand(EndEffectorConstants.FAST_TROUGH_SPEED, EndEffectorConstants.SLOW_TROUGH_SPEED)
                .until(coralDetectedTrigger.negate())
                .withName("End Effector Right Trough");
    }
}
