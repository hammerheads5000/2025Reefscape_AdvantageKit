// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.endeffector;

import static frc.robot.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants;
import frc.robot.Constants.EndEffectorConstants;

public class EndEffectorIOTalonFX implements EndEffectorIO {
    private final TalonFX leftMotor;
    private final TalonFX rightMotor;

    private final StatusSignal<AngularVelocity> leftVelocity;
    private final StatusSignal<AngularVelocity> rightVelocity;
    private final StatusSignal<Current> torqueCurrent;

    private final DigitalInput frontLidar;
    private final DigitalInput backLidar;
    private final DigitalInput intakeLidar;

    private final VoltageOut voltageRequest = new VoltageOut(0);
    private final NeutralOut neutralRequest = new NeutralOut();

    private final Debouncer leftConnectedDebouncer = new Debouncer(0.5);
    private final Debouncer rightConnectedDebouncer = new Debouncer(0.5);

    public EndEffectorIOTalonFX() {
        leftMotor = new TalonFX(EndEffectorConstants.MOTOR_LEFT_ID, Constants.CAN_FD_BUS);
        rightMotor = new TalonFX(EndEffectorConstants.MOTOR_RIGHT_ID, Constants.CAN_FD_BUS);

        tryUntilOk(5, () -> leftMotor.getConfigurator().apply(EndEffectorConstants.MOTOR_LEFT_CONFIGS));
        tryUntilOk(5, () -> rightMotor.getConfigurator().apply(EndEffectorConstants.MOTOR_RIGHT_CONFIGS));

        tryUntilOk(5, () -> leftMotor.getConfigurator().apply(EndEffectorConstants.CURRENT_LIMITS_CONFIGS));
        tryUntilOk(5, () -> rightMotor.getConfigurator().apply(EndEffectorConstants.CURRENT_LIMITS_CONFIGS));

        leftVelocity = leftMotor.getVelocity();
        rightVelocity = rightMotor.getVelocity();
        torqueCurrent = leftMotor.getTorqueCurrent();

        frontLidar = new DigitalInput(EndEffectorConstants.FRONT_LIDAR_ID);
        backLidar = new DigitalInput(EndEffectorConstants.BACK_LIDAR_ID);
        intakeLidar = new DigitalInput(EndEffectorConstants.INTAKE_LIDAR_ID);
    }

    @Override
    public void updateInputs(EndEffectorIOInputs inputs) {
        inputs.leftConnected = leftConnectedDebouncer.calculate(leftMotor.isConnected());
        inputs.rightConnected = rightConnectedDebouncer.calculate(rightMotor.isConnected());
        inputs.leftVelocity = leftVelocity.getValue();
        inputs.rightVelocity = rightVelocity.getValue();
        inputs.torqueCurrent = torqueCurrent.getValue();
        inputs.frontLidar = !frontLidar.get();
        inputs.backLidar = !backLidar.get();
        inputs.intakeLidar = !intakeLidar.get();
    }

    @Override
    public void setSpeeds(Voltage left, Voltage right) {
        leftMotor.setControl(voltageRequest.withOutput(left));
        rightMotor.setControl(voltageRequest.withOutput(right));
    }

    @Override
    public void stop() {
        leftMotor.setControl(neutralRequest);
        rightMotor.setControl(neutralRequest);
    }
}
