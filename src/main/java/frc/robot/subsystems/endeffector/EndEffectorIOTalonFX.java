// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.endeffector;

import static frc.robot.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANdi;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.EndEffectorConstants;

public class EndEffectorIOTalonFX implements EndEffectorIO {
    private final TalonFX leftMotor;
    private final TalonFX rightMotor;

    private final CANdi endEffectorCANdi; // front lidar S1, back on S2
    private final CANdi climberCANdi; // intake lidar on S2

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

        endEffectorCANdi = new CANdi(EndEffectorConstants.CANDI_ID, Constants.CAN_FD_BUS);
        climberCANdi = new CANdi(ClimberConstants.CANDI_ID, Constants.CAN_FD_BUS);
    }

    @Override
    public void updateInputs(EndEffectorIOInputs inputs) {
        inputs.leftConnected = leftConnectedDebouncer.calculate(leftMotor.isConnected());
        inputs.rightConnected = rightConnectedDebouncer.calculate(rightMotor.isConnected());
        inputs.leftVelocity = leftMotor.getVelocity().getValue();
        inputs.rightVelocity = rightMotor.getVelocity().getValue();
        inputs.torqueCurrent = leftMotor.getTorqueCurrent().getValue();
        inputs.frontLidar = endEffectorCANdi.getS1State().getValue() == EndEffectorConstants.S1_DETECTED_VALUE;
        inputs.backLidar = endEffectorCANdi.getS2State().getValue() == EndEffectorConstants.S2_DETECTED_VALUE;
        inputs.intakeLidar = climberCANdi.getS2State().getValue() == EndEffectorConstants.S2_DETECTED_VALUE;
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
