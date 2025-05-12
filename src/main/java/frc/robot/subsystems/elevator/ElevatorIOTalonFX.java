// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static frc.robot.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants;
import frc.robot.Constants.ElevatorConstants;

public class ElevatorIOTalonFX implements ElevatorIO {
    // Hardware
    private final TalonFX leadMotor = new TalonFX(ElevatorConstants.LEAD_MOTOR_ID, Constants.CAN_FD_BUS);
    private final TalonFX followMotor = new TalonFX(ElevatorConstants.FOLLOW_MOTOR_ID, Constants.CAN_FD_BUS);
    private final CANcoder encoder = new CANcoder(ElevatorConstants.ENCODER_ID, Constants.CAN_FD_BUS);

    private final Slot0Configs configs = ElevatorConstants.GAINS;

    // Status Signals
    private final StatusSignal<Angle> position;
    private final StatusSignal<AngularVelocity> velocity;
    private final StatusSignal<Voltage> appliedVoltage;
    private final StatusSignal<Current> torqueCurrent;
    private final StatusSignal<Double> setpointPosRads;
    private final StatusSignal<Double> setpointVelRadsPerSec;

    private final Debouncer leadConnectedDebouncer = new Debouncer(0.5);
    private final Debouncer followConnectedDebouncer = new Debouncer(0.5);
    private final Debouncer encoderConnectedDebouncer = new Debouncer(0.5);

    private final VoltageOut voltageRequest = new VoltageOut(0);
    private final MotionMagicExpoVoltage motionMagicRequest = new MotionMagicExpoVoltage(0);

    private boolean manualOverride = false;

    public ElevatorIOTalonFX() {
        followMotor.setControl(new Follower(leadMotor.getDeviceID(), ElevatorConstants.OPPOSE_FOLLOWER));

        tryUntilOk(5, () -> leadMotor.getConfigurator().apply(ElevatorConstants.MOTOR_CONFIGS, 0.25));

        position = leadMotor.getPosition();
        velocity = leadMotor.getVelocity();
        appliedVoltage = leadMotor.getMotorVoltage();
        torqueCurrent = leadMotor.getTorqueCurrent();
        setpointPosRads = leadMotor.getClosedLoopReference();
        setpointVelRadsPerSec = leadMotor.getClosedLoopReferenceSlope();
    }

    @Override
    public void updateInputs(ElevatorIOInputs inputs) {
        inputs.leadMotorConnected = leadConnectedDebouncer.calculate(leadMotor.isConnected());
        inputs.followMotorConnected = followConnectedDebouncer.calculate(followMotor.isConnected());
        inputs.encoderConnected = encoderConnectedDebouncer.calculate(encoder.isConnected());
        inputs.position = position.getValue();
        inputs.velocity = velocity.getValue();
        inputs.outputVoltage = appliedVoltage.getValue();
        inputs.outputCurrent = torqueCurrent.getValue();
        inputs.setpointPos = Radians.of(setpointPosRads.getValueAsDouble());
        inputs.setpointVel = RadiansPerSecond.of(setpointVelRadsPerSec.getValueAsDouble());
        inputs.manualOverride = manualOverride;
    }

    @Override
    public void setOpenLoopOutput(Voltage output) {
        if (!manualOverride) return;
        leadMotor.setControl(voltageRequest.withOutput(output));
    }

    @Override
    public void setGoal(Angle goal) {
        manualOverride = false;
        leadMotor.setControl(motionMagicRequest.withPosition(goal));
    }

    @Override
    public void resetAtPosition() {
        leadMotor.setControl(voltageRequest.withOutput(configs.kG));
    }

    @Override
    public void setPIDConstants(double kP, double kI, double kD, double kV, double kA, double kS, double kG) {
        configs.kP = kP;
        configs.kI = kI;
        configs.kD = kD;
        configs.kV = kV;
        configs.kA = kA;
        configs.kS = kS;
        configs.kG = kG;

        tryUntilOk(5, () -> leadMotor.getConfigurator().apply(configs));
    }

    @Override
    public void setManualOverride(boolean override) {
        manualOverride = override;
    }
}
