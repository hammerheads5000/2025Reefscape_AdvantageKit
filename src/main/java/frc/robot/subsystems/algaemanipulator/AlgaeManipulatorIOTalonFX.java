// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.algaemanipulator;

import static frc.robot.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants;
import frc.robot.Constants.AlgaeManipulatorConstants;

public class AlgaeManipulatorIOTalonFX implements AlgaeManipulatorIO {
    private final TalonFX motor;

    private final DigitalInput lidar;

    private final VoltageOut voltageRequest = new VoltageOut(0);
    private final NeutralOut neutralRequest = new NeutralOut();

    private final Debouncer connectedDebouncer = new Debouncer(0.5);

    public AlgaeManipulatorIOTalonFX() {
        motor = new TalonFX(AlgaeManipulatorConstants.MOTOR_ID, Constants.CAN_FD_BUS);

        tryUntilOk(5, () -> motor.getConfigurator().apply(AlgaeManipulatorConstants.MOTOR_CONFIGS));

        tryUntilOk(5, () -> motor.getConfigurator().apply(AlgaeManipulatorConstants.CURRENT_LIMITS_CONFIGS));

        lidar = new DigitalInput(AlgaeManipulatorConstants.LIDAR_ID);
    }

    @Override
    public void updateInputs(AlgaeManipulatorIOInputs inputs) {
        inputs.motorConnected = connectedDebouncer.calculate(motor.isConnected());
        inputs.velocity = motor.getVelocity().getValue();
        inputs.appliedVolts = motor.getMotorVoltage().getValue();
        inputs.torqueCurrent = motor.getTorqueCurrent().getValue();
        inputs.lidarSeesAlgae = !lidar.get();
    }

    @Override
    public void setSpeed(Voltage speed) {
        motor.setControl(voltageRequest.withOutput(speed));
    }

    @Override
    public void stop() {
        motor.setControl(neutralRequest);
    }
}
