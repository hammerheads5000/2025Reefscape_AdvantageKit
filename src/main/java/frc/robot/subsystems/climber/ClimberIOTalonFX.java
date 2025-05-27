// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climber;

import static frc.robot.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.CANdi;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants;
import frc.robot.Constants.ClimberConstants;

/** Add your docs here. */
public class ClimberIOTalonFX implements ClimberIO {
    private final TalonFX climbMotor;
    private final TalonFX grabMotor;

    private final CANdi cageSensor;

    private final Debouncer climbConnectedDebouncer = new Debouncer(0.5);
    private final Debouncer grabConnectedDebouncer = new Debouncer(0.5);
    private final Debouncer encoderConnectedDebouncer = new Debouncer(0.5);

    private final CANcoder climbEncoder;

    private final VoltageOut voltageRequest = new VoltageOut(0);
    private final NeutralOut neutralRequest = new NeutralOut();

    public ClimberIOTalonFX() {
        climbMotor = new TalonFX(ClimberConstants.CLIMB_MOTOR_ID, Constants.CAN_FD_BUS);
        grabMotor = new TalonFX(ClimberConstants.GRAB_MOTOR_ID, Constants.CAN_FD_BUS);
        tryUntilOk(5, () -> climbMotor.getConfigurator().apply(ClimberConstants.CLIMB_CONFIGS));
        tryUntilOk(5, () -> grabMotor.getConfigurator().apply(ClimberConstants.GRAB_CONFIGS));

        climbEncoder = new CANcoder(ClimberConstants.CLIMB_ENCODER_ID, Constants.CAN_FD_BUS);
        tryUntilOk(5, () -> climbEncoder.getConfigurator().apply(ClimberConstants.ENCODER_CONFIGS));

        cageSensor = new CANdi(ClimberConstants.CAGE_SENSOR_ID, Constants.CAN_FD_BUS);
    }

    @Override
    public void updateInputs(ClimberIOInputs inputs) {
        inputs.climbMotorConnected = climbConnectedDebouncer.calculate(climbMotor.isConnected());
        inputs.climbVelocity = climbMotor.getVelocity().getValue();
        inputs.climbAppliedVolts = climbMotor.getMotorVoltage().getValue();
        inputs.climbTorqueCurrent = climbMotor.getTorqueCurrent().getValue();

        inputs.grabMotorConnected = grabConnectedDebouncer.calculate(grabMotor.isConnected());
        inputs.grabVelocity = grabMotor.getVelocity().getValue();
        inputs.grabAppliedVolts = grabMotor.getMotorVoltage().getValue();

        inputs.encoderConnected = encoderConnectedDebouncer.calculate(climbEncoder.isConnected());
        inputs.pos = climbEncoder.getAbsolutePosition().getValue();

        inputs.cageDetected = cageSensor.getS1Closed().getValue();
    }

    @Override
    public void setClimberOutput(Voltage output) {
        climbMotor.setControl(voltageRequest.withOutput(output));
    }

    @Override
    public void stopClimb() {
        climbMotor.setControl(neutralRequest);
    }

    @Override
    public void setGrabOutput(Voltage output) {
        grabMotor.setControl(voltageRequest.withOutput(output));
    }

    @Override
    public void stopGrab() {
        grabMotor.setControl(neutralRequest);
    }
}
