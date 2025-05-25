// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.SignalLogger;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.PathConstants;
import frc.robot.util.LoggedTunableNumber;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Elevator extends SubsystemBase {
    private final ElevatorIO io;
    private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();

    private Distance goal = ElevatorConstants.INTAKE_HEIGHT;
    private final Debouncer atGoalDebouncer = new Debouncer(ElevatorConstants.AT_GOAL_DEBOUNCE_TIME.in(Seconds));

    private final Alert leadDisconnectedAlert;
    private final Alert followDisconnectedAlert;
    private final Alert encoderDisconnectedAlert;

    private final LoggedTunableNumber tolerance =
            new LoggedTunableNumber("Elevator/Tolerance", ElevatorConstants.TOLERANCE.in(Inches));

    private final SysIdRoutine sysIdRoutine;

    private final ElevatorVisualizer measuredVisualizer;
    private final ElevatorVisualizer setpointVisualizer;

    private final Trigger stageIs2 = new Trigger(() -> heightToStage(inputs.position) == Stage.STAGE2);

    public Elevator(
            ElevatorIO io,
            BooleanSupplier hasCoral,
            BooleanSupplier algaeDeployed,
            BooleanSupplier hasAlgae,
            Supplier<Pose2d> poseSupplier) {
        this.io = io;

        stageIs2.onChange(setStageCommand());

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

        measuredVisualizer = new ElevatorVisualizer("Measured", hasCoral, algaeDeployed, hasAlgae, poseSupplier);
        setpointVisualizer = new ElevatorVisualizer("Setpoint", hasCoral, algaeDeployed, hasAlgae, poseSupplier);

        SmartDashboard.putData("L1", goToL1Command(true));
        SmartDashboard.putData("L2", goToL2Command(true));
        SmartDashboard.putData("L3", goToL3Command(true));
        SmartDashboard.putData("L4", goToL4Command(true));
        SmartDashboard.putData("Intake", goToIntakePosCommand(true));
        SmartDashboard.putData("Zero Encoder", zeroEncoderCommand());
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Elevator", inputs);

        // Update alerts
        leadDisconnectedAlert.set(!inputs.leadMotorConnected);
        followDisconnectedAlert.set(!inputs.followMotorConnected);
        encoderDisconnectedAlert.set(!inputs.encoderConnected);

        // Logging
        Logger.recordOutput("Elevator/SetpointPosition", inputs.setpointPos);
        Logger.recordOutput("Elevator/SetpointVelocity", inputs.setpointVel);
        Logger.recordOutput("Elevator/Goal", goal);
        Logger.recordOutput("Elevator/Pos", inputs.position);
        Logger.recordOutput("Elevator/Stage", heightToStage(inputs.position));

        measuredVisualizer.update(inputs.position);
        setpointVisualizer.update(inputs.setpointPos);
    }

    public void setGoal(Distance goal) {
        this.goal = Meters.of(MathUtil.clamp(goal.in(Meters), 0, ElevatorConstants.MAX_HEIGHT.in(Meters)));
        io.setGoal(this.goal);
    }

    @AutoLogOutput
    public boolean atGoal() {
        return atGoalDebouncer.calculate(inputs.position.isNear(goal, Inches.of(tolerance.get())));
    }

    public void resetAtPosition() {
        this.goal = inputs.position;
        io.setManualOverride(true);
        io.resetAtPosition();
    }

    public static Distance encoderAngleToHeight(Angle angle) {
        return Meters.of(
                ElevatorConstants.MIN_HEIGHT.in(Meters) + angle.in(Radians) * ElevatorConstants.DRUM_RADIUS.in(Meters));
    }

    public static Angle heightToEncoderAngle(Distance height) {
        return Radians.of((height.in(Meters) - ElevatorConstants.MIN_HEIGHT.in(Meters))
                / ElevatorConstants.DRUM_RADIUS.in(Meters));
    }

    public Command zeroEncoderCommand() {
        return this.runOnce(io::zeroEncoder);
    }

    public Command goToHeightCommand(boolean instant, Distance goal) {
        if (instant) {
            return this.runOnce(() -> setGoal(goal));
        }
        return this.startEnd(() -> setGoal(goal), () -> {}).until(this::atGoal).handleInterrupt(this::resetAtPosition);
    }

    public Command goToL1Command(boolean instant) {
        return goToHeightCommand(instant, ElevatorConstants.L1_HEIGHT)
                .andThen(Commands.waitTime(PathConstants.ELEVATOR_SETTLE_TIME));
    }

    public Command goToL2Command(boolean instant) {
        return goToHeightCommand(instant, ElevatorConstants.L2_HEIGHT)
                .andThen(Commands.waitTime(PathConstants.ELEVATOR_SETTLE_TIME));
    }

    public Command goToL3Command(boolean instant) {
        return goToHeightCommand(instant, ElevatorConstants.L3_HEIGHT)
                .andThen(Commands.waitTime(PathConstants.ELEVATOR_SETTLE_TIME));
    }

    public Command goToL4Command(boolean instant) {
        return goToHeightCommand(instant, ElevatorConstants.L4_HEIGHT);
    }

    public Command goToIntakePosCommand(boolean instant) {
        return goToHeightCommand(instant, ElevatorConstants.INTAKE_HEIGHT);
    }

    public Command goToAlgaeCommand(int side, boolean instant) {
        return goToHeightCommand(instant, ElevatorConstants.ALGAE_HEIGHTS.get(side));
    }

    public Command goToBargeCommand(boolean instant) {
        return goToHeightCommand(instant, ElevatorConstants.BARGE_HEIGHT);
    }

    public Command trackHeightCommand(Supplier<Distance> distanceToReef, Distance height) {
        return this.run(() -> {
                    Distance dist = Meters.of(Math.min(
                            ElevatorConstants.MAX_SHOOT_DISTANCE.in(Meters),
                            distanceToReef.get().in(Meters)));
                    Distance additionalHeight = dist.times(Math.tan(ElevatorConstants.SHOOT_ANGLE.in(Radians)));
                    setGoal(height.plus(additionalHeight));
                })
                .handleInterrupt(this::resetAtPosition);
    }

    public Command trackL1Command(Supplier<Distance> distanceToReef) {
        return trackHeightCommand(distanceToReef, ElevatorConstants.L1_HEIGHT);
    }

    public Command trackL2Command(Supplier<Distance> distanceToReef) {
        return trackHeightCommand(distanceToReef, ElevatorConstants.L2_HEIGHT);
    }

    public Command trackL3Command(Supplier<Distance> distanceToReef) {
        return trackHeightCommand(distanceToReef, ElevatorConstants.L3_HEIGHT);
    }

    public Command trackL4Command(Supplier<Distance> distanceToReef) {
        return trackHeightCommand(distanceToReef, ElevatorConstants.L4_HEIGHT);
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

    private Command setStageCommand() {
        return Commands.runOnce(() -> io.setStage(heightToStage(inputs.position)));
    }

    public enum Stage {
        STAGE3(0), // end effector, first to move
        STAGE2(1), // middle stage, second to move
        STAGE1(2); // final stage to move

        public int slot;

        private Stage(int slot) {
            this.slot = slot;
        }
    }

    public static Stage heightToStage(Distance height) {
        if (height.gt(ElevatorConstants.STAGE1_HEIGHT)) {
            return Stage.STAGE1;
        }
        if (height.gt(ElevatorConstants.STAGE2_HEIGHT)) {
            return Stage.STAGE2;
        }
        return Stage.STAGE3;
    }

    public static Stage levelToStage(char level) {
        Distance height;
        switch (level) {
            case '1':
                height = ElevatorConstants.L1_HEIGHT;
                break;
            case '2':
                height = ElevatorConstants.L2_HEIGHT;
                break;
            case '3':
                height = ElevatorConstants.L3_HEIGHT;
                break;
            case '4':
                height = ElevatorConstants.L4_HEIGHT;
                break;
            default:
                height = ElevatorConstants.L4_HEIGHT;
                break;
        }

        return heightToStage(height);
    }
}
