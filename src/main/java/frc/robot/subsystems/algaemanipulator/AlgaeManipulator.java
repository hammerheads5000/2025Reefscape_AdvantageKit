// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.algaemanipulator;

import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.Constants.AlgaeManipulatorConstants;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

/**
 * Button 2 on button board 2 if true remove algae while doing coral Y is remove algae no coral X is barge - depends on
 * F,G,H,I for pose To remove coral, back up, flip down + move elevator, move forward, wait for pickup, move back, drop
 */
public class AlgaeManipulator extends SubsystemBase {
    private final AlgaeManipulatorIO io;
    private final AlgaeManipulatorIOInputsAutoLogged inputs = new AlgaeManipulatorIOInputsAutoLogged();

    private Trigger isSim = new Trigger(() -> Constants.CURRENT_MODE == Constants.SIM_MODE);
    private boolean simHasAlgae = false;

    @AutoLogOutput
    public Trigger algaeDetectedTrigger =
            new Trigger(() -> inputs.lidarSeesAlgae).debounce(0.1).or(isSim.and(() -> simHasAlgae));

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

    private Command runCommand(Voltage speed) {
        return this.startEnd(() -> io.setSpeed(speed), io::stop);
    }

    public Command intakeCommand() {
        if (isSim.getAsBoolean()) {
            return this.runOnce(() -> simHasAlgae = true);
        }
        return runCommand(AlgaeManipulatorConstants.INTAKE_SPEED)
                .until(algaeDetectedTrigger)
                .andThen(this.runOnce(() -> io.setSpeed(AlgaeManipulatorConstants.HOLD_SPEED)));
    }

    public Command reverseCommand() {
        return runCommand(AlgaeManipulatorConstants.EJECT_SPEED.unaryMinus());
    }

    public Command ejectCommand() {
        if (isSim.getAsBoolean()) {
            return this.runOnce(() -> simHasAlgae = false);
        }
        return reverseCommand().until(algaeDetectedTrigger.negate());
    }

    public Command flipUpAndHoldCommand() {
        return runCommand(AlgaeManipulatorConstants.FLIP_UP_SPEED)
                .withTimeout(AlgaeManipulatorConstants.FLIP_UP_TIME)
                .andThen(this.runOnce(() -> io.setSpeed(AlgaeManipulatorConstants.HOLD_UP_SPEED)));
    }
}
