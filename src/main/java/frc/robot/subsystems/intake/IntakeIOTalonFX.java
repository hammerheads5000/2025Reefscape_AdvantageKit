// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static frc.robot.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.CANdi;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.S1StateValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants;
import org.littletonrobotics.junction.Logger;

/** Add your docs here. */
public class IntakeIOTalonFX implements IntakeIO {
    private final TalonFX intakeMotor;
    private final TalonFX alignMotor;
    private final TalonFX deployMotor;

    private final CANcoder encoder;

    private final CANdi candi;
    private final StatusSignal<S1StateValue> s1StateSignal;
    private boolean lastS1High = false;
    private boolean hasReportedCandiError = false;

    private final MotionMagicVoltage deployRequest = new MotionMagicVoltage(0);
    private final VoltageOut intakeRequest = new VoltageOut(0);
    private final VoltageOut alignRequest = new VoltageOut(0);

    public IntakeIOTalonFX() {
        intakeMotor = new TalonFX(IntakeConstants.INTAKE_MOTOR_ID, Constants.CAN_FD_BUS);
        alignMotor = new TalonFX(IntakeConstants.ALIGN_MOTOR_ID, Constants.CAN_FD_BUS);
        deployMotor = new TalonFX(IntakeConstants.DEPLOY_MOTOR_ID, Constants.CAN_FD_BUS);

        encoder = new CANcoder(IntakeConstants.ENCODER_ID, Constants.CAN_FD_BUS);

        CANdi canDevice = null;
        StatusSignal<S1StateValue> signal = null;
        try {
            canDevice = new CANdi(IntakeConstants.INTAKE_CANDI_ID, Constants.CAN_FD_BUS);
            signal = canDevice.getS1State();
            StatusCode freqStatus = signal.setUpdateFrequency(100.0);
            if (!freqStatus.isOK()) {
                DriverStation.reportWarning("Failed to set CANdi S1 update rate: " + freqStatus, false);
            }
        } catch (Exception e) {
            DriverStation.reportWarning("Failed to initialize CANdi for intake lidar: " + e.getMessage(), false);
        }
        candi = canDevice;
        s1StateSignal = signal;

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

        if (s1StateSignal != null) {
            StatusCode status = BaseStatusSignal.refreshAll(s1StateSignal);
            if (status.isOK()) {
                lastS1High = s1StateSignal.getValue() == S1StateValue.High;
                hasReportedCandiError = false;
            } else if (!hasReportedCandiError) {
                DriverStation.reportWarning("CANdi S1 refresh failed: " + status, false);
                hasReportedCandiError = true;
            }
        }

        inputs.alignLidar = lastS1High;

        inputs.setpointPos = Rotations.of(deployMotor.getClosedLoopReference().getValue());
        inputs.setpointVel =
                RotationsPerSecond.of(deployMotor.getClosedLoopReferenceSlope().getValue());
    }

    @Override
    public void setGoal(Angle angle) {
        Logger.recordOutput("Intake/Goal(deg)", angle.in(Degrees));
        deployMotor.setControl(deployRequest.withPosition(angle));
    }

    @Override
    public void setIntakeSpeed(Voltage speed) {
        Logger.recordOutput("Intake/Intake Speed (V)", speed);
        intakeMotor.setControl(intakeRequest.withOutput(speed));
    }

    @Override
    public void setAlignSpeed(Voltage speed) {
        alignMotor.setControl(alignRequest.withOutput(speed));
    }
}
