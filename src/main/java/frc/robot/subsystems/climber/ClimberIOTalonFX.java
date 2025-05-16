// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climber;

import static frc.robot.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants;
import frc.robot.Constants.ClimberConstants;

/** Add your docs here. */
public class ClimberIOTalonFX implements ClimberIO {
    private final TalonFX motor;

    private final Debouncer connectedDebouncer = new Debouncer(0.5);

    private final StatusSignal<AngularVelocity> velocity;
    private final StatusSignal<Current> torqueCurrent;

    private final VoltageOut voltageRequest = new VoltageOut(0);
    private final NeutralOut neutralRequest = new NeutralOut();

    public ClimberIOTalonFX() {
        motor = new TalonFX(ClimberConstants.MOTOR_ID, Constants.CAN_FD_BUS);
        tryUntilOk(5, () -> motor.getConfigurator().apply(ClimberConstants.OUTPUT_CONFIGS));
        tryUntilOk(5, () -> motor.getConfigurator().apply(ClimberConstants.CURRENT_LIMITS_CONFIGS));

        velocity = motor.getVelocity();
        torqueCurrent = motor.getTorqueCurrent();
    }

    @Override
    public void updateInputs(ClimberIOInputs inputs) {
        inputs.motorConnected = connectedDebouncer.calculate(motor.isConnected());
        inputs.velocity = velocity.getValue();
        inputs.torqueCurrent = torqueCurrent.getValue();
    }

    @Override
    public void setOutput(Voltage output) {
        motor.setControl(voltageRequest.withOutput(output));
    }

    @Override
    public void stop() {
        motor.setControl(neutralRequest);
    }
}
