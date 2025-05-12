// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.SignalLogger;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.util.LoggedTunableNumber;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Elevator extends SubsystemBase {
    private final ElevatorIO io;
    private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();

    private Angle goal = ElevatorConstants.INTAKE_HEIGHT;
    private final Debouncer atGoalDebouncer = new Debouncer(ElevatorConstants.AT_GOAL_DEBOUNCE_TIME.in(Seconds));

    private final Alert leadDisconnectedAlert;
    private final Alert followDisconnectedAlert;
    private final Alert encoderDisconnectedAlert;

    // Tunable parameters (radians)
    private final LoggedTunableNumber kP = new LoggedTunableNumber("Elevator/kP", ElevatorConstants.GAINS.kP);
    private final LoggedTunableNumber kI = new LoggedTunableNumber("Elevator/kI", ElevatorConstants.GAINS.kI);
    private final LoggedTunableNumber kD = new LoggedTunableNumber("Elevator/kD", ElevatorConstants.GAINS.kD);
    private final LoggedTunableNumber kA = new LoggedTunableNumber("Elevator/kA", ElevatorConstants.GAINS.kA);
    private final LoggedTunableNumber kV = new LoggedTunableNumber("Elevator/kV", ElevatorConstants.GAINS.kV);
    private final LoggedTunableNumber kS = new LoggedTunableNumber("Elevator/kS", ElevatorConstants.GAINS.kS);
    private final LoggedTunableNumber kG = new LoggedTunableNumber("Elevator/kG", ElevatorConstants.GAINS.kG);
    private final LoggedTunableNumber tolerance =
            new LoggedTunableNumber("Elevator/Tolerance", ElevatorConstants.TOLERANCE.in(Radians));

    private final SysIdRoutine sysIdRoutine;

    private final ElevatorVisualizer measuredVisualizer;
    private final ElevatorVisualizer setpointVisualizer;

    public Elevator(ElevatorIO io, BooleanSupplier hasCoral, Supplier<Pose2d> poseSupplier) {
        this.io = io;

        leadDisconnectedAlert = new Alert("Disconnected elevator lead motor.", Alert.AlertType.kError);
        followDisconnectedAlert = new Alert("Disconnected elevator follow motor.", Alert.AlertType.kError);
        encoderDisconnectedAlert = new Alert("Disconnected elevator encoder.", Alert.AlertType.kError);

        sysIdRoutine = new SysIdRoutine(
                new SysIdRoutine.Config(
                        Volts.of(0.5).per(Second),
                        Volts.of(1.5),
                        null,
                        state -> SignalLogger.writeString("SysId_Elevator_State", state.toString())),
                new SysIdRoutine.Mechanism(output -> io.setOpenLoopOutput(output), null, this));

        measuredVisualizer = new ElevatorVisualizer("Measured", hasCoral, poseSupplier);
        setpointVisualizer = new ElevatorVisualizer("Setpoint", hasCoral, poseSupplier);

        SmartDashboard.putData("L1", goToL1Command(true));
        SmartDashboard.putData("L2", goToL2Command(true));
        SmartDashboard.putData("L3", goToL3Command(true));
        SmartDashboard.putData("L4", goToL4Command(true));
        SmartDashboard.putData("Intake", goToIntakePosCommand(true));
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Elevator", inputs);

        // Update alerts
        leadDisconnectedAlert.set(!inputs.leadMotorConnected);
        followDisconnectedAlert.set(!inputs.followMotorConnected);
        encoderDisconnectedAlert.set(!inputs.encoderConnected);

        // Update tunable parameters
        if (kP.hasChanged(hashCode())
                || kI.hasChanged(hashCode())
                || kD.hasChanged(hashCode())
                || kA.hasChanged(hashCode())
                || kV.hasChanged(hashCode())
                || kS.hasChanged(hashCode())
                || kG.hasChanged(hashCode())) {
            io.setPIDConstants(kP.get(), kI.get(), kD.get(), kV.get(), kA.get(), kS.get(), kG.get());
        }

        // Logging
        Logger.recordOutput("Elevator/SetpointPosition", inputs.setpointPos);
        Logger.recordOutput("Elevator/SetpointVelocity", inputs.setpointVel);
        Logger.recordOutput("Elevator/Goal", goal);

        measuredVisualizer.update(angleToHeight(inputs.position));
        setpointVisualizer.update(angleToHeight(inputs.setpointPos));
    }

    public void setGoal(Angle goal) {
        this.goal = goal;
        io.setGoal(goal);
    }

    @AutoLogOutput
    public boolean atGoal() {
        return atGoalDebouncer.calculate(inputs.position.isNear(goal, Radians.of(tolerance.get())));
    }

    public void resetAtPosition() {
        this.goal = inputs.position;
        io.setManualOverride(true);
        io.resetAtPosition();
    }

    private Distance angleToHeight(Angle angle) {
        return Meters.of(ElevatorConstants.MIN_HEIGHT.in(Meters)
                + angle.in(Radians) * ElevatorConstants.DRUM_RADIUS.in(Meters) * 3);
    }

    public Command goToHeightCommand(boolean instant, Angle goal) {
        if (instant) {
            return this.runOnce(() -> setGoal(goal));
        }
        return this.startEnd(() -> setGoal(goal), () -> {}).until(this::atGoal).handleInterrupt(this::resetAtPosition);
    }

    public Command goToL1Command(boolean instant) {
        return goToHeightCommand(instant, ElevatorConstants.L1_HEIGHT)
                .andThen(Commands.waitTime(ElevatorConstants.ELEVATOR_SETTLE_TIME));
    }

    public Command goToL2Command(boolean instant) {
        return goToHeightCommand(instant, ElevatorConstants.L2_HEIGHT)
                .andThen(Commands.waitTime(ElevatorConstants.ELEVATOR_SETTLE_TIME));
    }

    public Command goToL3Command(boolean instant) {
        return goToHeightCommand(instant, ElevatorConstants.L3_HEIGHT)
                .andThen(Commands.waitTime(ElevatorConstants.ELEVATOR_SETTLE_TIME));
    }

    public Command goToL4Command(boolean instant) {
        return goToHeightCommand(instant, ElevatorConstants.L4_HEIGHT);
    }

    public Command goToIntakePosCommand(boolean instant) {
        return goToHeightCommand(instant, ElevatorConstants.INTAKE_HEIGHT);
    }

    public Command elevatorUpCommand() {
        return this.startEnd(
                () -> {
                    io.setManualOverride(true);
                    io.setOpenLoopOutput(ElevatorConstants.MANUAL_UP_SPEED);
                },
                io::resetAtPosition);
    }

    public Command elevatorDownCommand() {
        return this.startEnd(
                () -> {
                    io.setManualOverride(true);
                    io.setOpenLoopOutput(ElevatorConstants.MANUAL_DOWN_SPEED);
                },
                io::resetAtPosition);
    }

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return sysIdRoutine.quasistatic(direction);
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return sysIdRoutine.dynamic(direction);
    }
}
