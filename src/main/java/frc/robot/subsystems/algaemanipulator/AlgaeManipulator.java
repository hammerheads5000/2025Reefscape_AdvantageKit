// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.algaemanipulator;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.Constants.AlgaeManipulatorConstants;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class AlgaeManipulator extends SubsystemBase {
    private final AlgaeManipulatorIO io;
    private final AlgaeManipulatorIOInputsAutoLogged inputs = new AlgaeManipulatorIOInputsAutoLogged();

    private Trigger isSim = new Trigger(() -> Constants.CURRENT_MODE == Constants.SIM_MODE);
    private boolean simHasAlgae = false;
    private boolean deployed = false; // whether bottom part of the manipulator is down or not

    @AutoLogOutput
    public Trigger algaeDetectedTrigger =
            new Trigger(this::stalled).debounce(0.4).or(isSim.and(() -> simHasAlgae));

    @AutoLogOutput
    public Trigger deployedTrigger = new Trigger(() -> deployed);

    private final Alert motorDisconnectedAlert = new Alert("Disconnected algae manipulator motor.", AlertType.kError);

    public AlgaeManipulator(AlgaeManipulatorIO io) {
        this.io = io;

        SmartDashboard.putData("Algae Intake", intakeCommand());
        SmartDashboard.putData("Eject Algae", ejectCommand());
        SmartDashboard.putData("Algae Forward", forwardCommand());
        SmartDashboard.putData("Algae Reverse", reverseCommand());
        SmartDashboard.putData("Algae Flip Up and Hold", flipUpAndHoldCommand());
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("AlgaeManipulator", inputs);

        motorDisconnectedAlert.set(!inputs.motorConnected);
    }

    public void stop() {
        io.stop();
        this.getCurrentCommand().cancel();
    }

    public Command stopCommand() {
        return this.runOnce(this::stop);
    }

    /** Set speed and immediately end */
    public Command setSpeedCommand(Voltage speed) {
        return this.runOnce(() -> io.setSpeed(speed));
    }

    /** Intake algae. Ends when algae is detected, but continues with intakeCycleCommand to hold algae */
    public Command intakeCommand() {
        return Commands.sequence(
                        this.runOnce(() -> {
                            simHasAlgae = true;
                            deployed = true;
                        }),
                        setSpeedCommand(AlgaeManipulatorConstants.INTAKE_SPEED),
                        Commands.waitUntil(algaeDetectedTrigger),
                        new ScheduleCommand(intakeCycleCommand()))
                .withName("Algae Intake");
    }

    /** Holds algae in with short cycles of max speed and longer cycles of slower speed */
    private Command intakeCycleCommand() {
        return Commands.waitTime(AlgaeManipulatorConstants.HOLD_TIME)
                .andThen(Commands.repeatingSequence(
                        setSpeedCommand(AlgaeManipulatorConstants.HOLD_SPEED),
                        Commands.waitTime(AlgaeManipulatorConstants.HOLD_CYCLE_OFF),
                        setSpeedCommand(AlgaeManipulatorConstants.INTAKE_SPEED),
                        Commands.waitTime(AlgaeManipulatorConstants.HOLD_CYCLE_ON)))
                .withName("Algae Intake Cycle");
    }

    /** Manually move algae manipulator motor forward (intake) and stop on interrupt */
    public Command forwardCommand() {
        return setSpeedCommand(AlgaeManipulatorConstants.INTAKE_SPEED)
                .andThen(Commands.idle(this))
                .finallyDo(this::stop)
                .withName("Algae Forward");
    }

    /** Manually move algae manipulator motor backward (eject) and stop on interrupt */
    public Command reverseCommand() {
        return setSpeedCommand(AlgaeManipulatorConstants.EJECT_SPEED)
                .andThen(Commands.idle(this))
                .finallyDo(this::stop)
                .withName("Algae Reverse");
    }

    /** Eject algae by running the motor backwards until algae is no longer detected */
    public Command ejectCommand() {
        return this.runOnce(() -> simHasAlgae = false)
                .andThen(reverseCommand().until(algaeDetectedTrigger.negate()))
                .withName("Algae Eject");
    }

    /** Stows the algae manipulator by flipping it up and holding it in place (ends naturally after FLIP_UP_TIME) */
    public Command flipUpAndHoldCommand() {
        return Commands.sequence(
                        this.runOnce(() -> deployed = false),
                        setSpeedCommand(AlgaeManipulatorConstants.FLIP_UP_SPEED),
                        Commands.waitTime(AlgaeManipulatorConstants.FLIP_UP_TIME),
                        setSpeedCommand(AlgaeManipulatorConstants.HOLD_UP_SPEED))
                .withName("Flip up and Hold");
    }

    /** Detects whether motor is stalled (not moving and has sufficient current) to determine if it is holding algae */
    private boolean stalled() {
        return inputs.velocity.abs(RotationsPerSecond) < AlgaeManipulatorConstants.MIN_VEL.abs(RotationsPerSecond)
                && inputs.torqueCurrent.abs(Amps) >= AlgaeManipulatorConstants.STALL_CURRENT.abs(Amps);
    }
}
