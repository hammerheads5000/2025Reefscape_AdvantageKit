// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static frc.robot.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants;

/** Add your docs here. */
public class IntakeIOTalonFX implements IntakeIO {
    private final TalonFX intakeMotor;
    private final TalonFX alignMotor;
    private final TalonFX deployMotor;

    private final CANcoder encoder;

    private final DigitalInput alignLidar = new DigitalInput(IntakeConstants.ALIGN_LIDAR_ID);

    private final PositionVoltage deployRequest = new PositionVoltage(0);
    private final VoltageOut voltageRequets = new VoltageOut(0);

    public IntakeIOTalonFX() {
        intakeMotor = new TalonFX(IntakeConstants.INTAKE_MOTOR_ID, Constants.CAN_FD_BUS);
        alignMotor = new TalonFX(IntakeConstants.ALIGN_MOTOR_ID, Constants.CAN_FD_BUS);
        deployMotor = new TalonFX(IntakeConstants.DEPLOY_MOTOR_ID, Constants.CAN_FD_BUS);

        encoder = new CANcoder(IntakeConstants.ENCODER_ID, Constants.CAN_FD_BUS);

        tryUntilOk(5, () -> intakeMotor.getConfigurator().apply(IntakeConstants.INTAKE_MOTOR_CONFIGS));
        tryUntilOk(5, () -> alignMotor.getConfigurator().apply(IntakeConstants.ALIGN_MOTOR_CONFIGS));
        tryUntilOk(5, () -> deployMotor.getConfigurator().apply(IntakeConstants.DEPLOY_MOTOR_CONFIGS));

        tryUntilOk(5, () -> encoder.getConfigurator().apply(IntakeConstants.ENCODER_CONFIGS));
    }

    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        inputs.intakeMotorConnected = intakeMotor.isConnected();
        inputs.alignMotorConnected = alignMotor.isConnected();
        inputs.deployMotorConnected = deployMotor.isConnected();

        inputs.intakeVelocity = intakeMotor.getVelocity().getValue();
        inputs.alignVelocity = alignMotor.getVelocity().getValue();
        inputs.deployVelocity = deployMotor.getVelocity().getValue();

        inputs.position = encoder.getPosition().getValue();

        inputs.intakeCurrent = intakeMotor.getTorqueCurrent().getValue();
        inputs.alignCurrent = alignMotor.getTorqueCurrent().getValue();
        inputs.deployTorqueCurrent = deployMotor.getTorqueCurrent().getValue();

        inputs.alignLidar = alignLidar.get();

        inputs.setpointPos = Rotations.of(deployMotor.getClosedLoopReference().getValue());
        inputs.setpointVel =
                RotationsPerSecond.of(deployMotor.getClosedLoopReferenceSlope().getValue());
    }

    @Override
    public void setGoal(Angle angle) {
        deployMotor.setControl(deployRequest.withPosition(angle));
    }

    @Override
    public void setIntakeSpeed(Voltage speed) {
        intakeMotor.setControl(voltageRequets.withOutput(speed));
    }

    @Override
    public void setAlignSpeed(Voltage speed) {
        alignMotor.setControl(voltageRequets.withOutput(speed));
    }
}
