// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.algaemanipulator;

import edu.wpi.first.math.filter.Debouncer.DebounceType;
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
    private boolean deployed = false;

    @AutoLogOutput
    public Trigger algaeDetectedTrigger =
            new Trigger(this::stalled).debounce(0.4, DebounceType.kFalling).or(isSim.and(() -> simHasAlgae));

    @AutoLogOutput
    public Trigger deployedTrigger = new Trigger(() -> deployed);

    private final Alert motorDisconnectedAlert = new Alert("Disconnected algae manipulator motor.", AlertType.kError);

    public AlgaeManipulator(AlgaeManipulatorIO io) {
        this.io = io;

        SmartDashboard.putData("Algae Intake", intakeCommand());
        SmartDashboard.putData("Eject Algae", ejectCommand());
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
    }

    public Command stopCommand() {
        return this.runOnce(this::stop);
    }

    public Command setSpeedCommand(Voltage speed, boolean requireSubsystem) {
        if (!requireSubsystem) {
            return Commands.runOnce(() -> io.setSpeed(speed));
        }
        return this.runOnce(() -> io.setSpeed(speed));
    }

    public Command setSpeedCommand(Voltage speed) {
        return setSpeedCommand(speed, true);
    }

    public Command intakeCommand() {
        return Commands.sequence(
                        this.runOnce(() -> {
                            simHasAlgae = true;
                            deployed = true;
                        }),
                        setSpeedCommand(AlgaeManipulatorConstants.INTAKE_SPEED),
                        Commands.waitUntil(algaeDetectedTrigger),
                        setSpeedCommand(AlgaeManipulatorConstants.INTAKE_SPEED),
                        new ScheduleCommand(Commands.waitTime(AlgaeManipulatorConstants.HOLD_TIME)
                                .andThen(intakeCycleCommand())))
                .withName("Algae Intake");
    }

    private Command intakeCycleCommand() {
        return Commands.repeatingSequence(
                        setSpeedCommand(AlgaeManipulatorConstants.HOLD_SPEED),
                        Commands.waitTime(AlgaeManipulatorConstants.HOLD_CYCLE_OFF),
                        setSpeedCommand(AlgaeManipulatorConstants.INTAKE_SPEED),
                        Commands.waitTime(AlgaeManipulatorConstants.HOLD_CYCLE_ON))
                .withName("Algae Intake Cycle");
    }

    public Command forwardCommand() {
        return setSpeedCommand(AlgaeManipulatorConstants.INTAKE_SPEED)
                .andThen(Commands.idle(this))
                .finallyDo(this::stop);
    }

    public Command reverseCommand() {
        return setSpeedCommand(AlgaeManipulatorConstants.EJECT_SPEED)
                .andThen(Commands.idle(this))
                .finallyDo(this::stop);
    }

    public Command ejectCommand() {
        return this.runOnce(() -> simHasAlgae = false).andThen(reverseCommand().until(algaeDetectedTrigger.negate()));
    }

    public Command flipUpAndHoldCommand() {
        return Commands.sequence(
                this.runOnce(() -> deployed = false),
                setSpeedCommand(AlgaeManipulatorConstants.FLIP_UP_SPEED),
                Commands.waitTime(AlgaeManipulatorConstants.FLIP_UP_TIME),
                setSpeedCommand(AlgaeManipulatorConstants.HOLD_UP_SPEED));
    }

    private boolean stalled() {
        return inputs.velocity.lt(AlgaeManipulatorConstants.MIN_VEL)
                && inputs.torqueCurrent.gt(AlgaeManipulatorConstants.STALL_CURRENT);
    }
}
