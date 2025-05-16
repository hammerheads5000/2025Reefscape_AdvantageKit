// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static frc.robot.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.configs.SlotConfigs;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.subsystems.elevator.Elevator.Stage;

public class ElevatorIOTalonFX implements ElevatorIO {
    // Hardware
    private final TalonFX leadMotor = new TalonFX(ElevatorConstants.LEAD_MOTOR_ID, Constants.CAN_FD_BUS);
    private final TalonFX followMotor = new TalonFX(ElevatorConstants.FOLLOW_MOTOR_ID, Constants.CAN_FD_BUS);
    private final CANcoder encoder = new CANcoder(ElevatorConstants.ENCODER_ID, Constants.CAN_FD_BUS);

    private final Debouncer leadConnectedDebouncer = new Debouncer(0.5);
    private final Debouncer followConnectedDebouncer = new Debouncer(0.5);
    private final Debouncer encoderConnectedDebouncer = new Debouncer(0.5);

    private final VoltageOut voltageRequest = new VoltageOut(0);
    private final MotionMagicExpoVoltage motionMagicRequest = new MotionMagicExpoVoltage(0);

    private boolean manualOverride = false;

    private Stage stage = Stage.STAGE3;
    private Distance goal = ElevatorConstants.INTAKE_HEIGHT;

    public ElevatorIOTalonFX() {
        followMotor.setControl(new Follower(leadMotor.getDeviceID(), ElevatorConstants.OPPOSE_FOLLOWER));

        tryUntilOk(5, () -> leadMotor.getConfigurator().apply(ElevatorConstants.MOTOR_CONFIGS, 0.25));
    }

    @Override
    public void updateInputs(ElevatorIOInputs inputs) {
        inputs.leadMotorConnected = leadConnectedDebouncer.calculate(leadMotor.isConnected());
        inputs.followMotorConnected = followConnectedDebouncer.calculate(followMotor.isConnected());
        inputs.encoderConnected = encoderConnectedDebouncer.calculate(encoder.isConnected());
        inputs.position = Elevator.encoderAngleToHeight(leadMotor.getPosition().getValue());
        inputs.velocity = MetersPerSecond.of(
                leadMotor.getVelocity().getValue().in(RadiansPerSecond) * ElevatorConstants.DRUM_RADIUS.in(Meters));
        inputs.outputVoltage = leadMotor.getMotorVoltage().getValue();
        inputs.outputCurrent = leadMotor.getTorqueCurrent().getValue();
        inputs.setpointPos = Elevator.encoderAngleToHeight(
                Rotations.of(leadMotor.getClosedLoopReference().getValueAsDouble()));
        inputs.setpointVel = MetersPerSecond.of(
                leadMotor.getClosedLoopReferenceSlope().getValue() * ElevatorConstants.DRUM_RADIUS.in(Meters));
        inputs.manualOverride = manualOverride;
    }

    @Override
    public void setOpenLoopOutput(Voltage output) {
        if (!manualOverride) return;
        leadMotor.setControl(voltageRequest.withOutput(output));
    }

    @Override
    public void setGoal(Distance goal) {
        manualOverride = false;
        leadMotor.setControl(motionMagicRequest
                .withPosition(Elevator.heightToEncoderAngle(goal))
                .withSlot(stage.slot));
        this.goal = goal;
    }

    @Override
    public void resetAtPosition() {
        leadMotor.setControl(voltageRequest.withOutput(getStageGains(stage).kG));
    }

    @Override
    public void setManualOverride(boolean override) {
        manualOverride = override;
    }

    @Override
    public void setStage(Stage stage) {
        this.stage = stage;

        if (manualOverride) return;
        leadMotor.setControl(motionMagicRequest
                .withPosition(Elevator.heightToEncoderAngle(goal))
                .withSlot(stage.slot));
    }

    public static SlotConfigs getStageGains(Stage currentStage) {
        switch (currentStage) {
            case STAGE1:
                return SlotConfigs.from(ElevatorConstants.STAGE1_GAINS);
            case STAGE2:
                return SlotConfigs.from(ElevatorConstants.STAGE2_GAINS);
            case STAGE3:
                return SlotConfigs.from(ElevatorConstants.STAGE3_GAINS);
            default:
                throw new IllegalArgumentException("Invalid stage: " + currentStage);
        }
    }
}
