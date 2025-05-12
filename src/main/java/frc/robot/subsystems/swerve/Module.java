// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.swerve;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import org.littletonrobotics.junction.Logger;

/** Add your docs here. */
public class Module {
    private final ModuleIO io;
    private final ModuleIOInputsAutoLogged inputs = new ModuleIOInputsAutoLogged();
    private final int index;
    private final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> constants;

    private final Alert driveDisconnectedAlert;
    private final Alert steerDisconnectedAlert;
    private final Alert steerEncoderDisconnectedAlert;
    private SwerveModulePosition[] odometryPositions = new SwerveModulePosition[] {};

    public Module(
            ModuleIO io,
            int index,
            SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> constants) {
        this.io = io;
        this.index = index;
        this.constants = constants;

        driveDisconnectedAlert =
                new Alert("Disconnected drive motor on module " + Integer.toString(index) + ".", AlertType.kError);
        steerDisconnectedAlert =
                new Alert("Disconnected turn motor on module " + Integer.toString(index) + ".", AlertType.kError);
        steerEncoderDisconnectedAlert =
                new Alert("Disconnected turn encoder on module " + Integer.toString(index) + ".", AlertType.kError);
    }

    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Swerve/Module" + index, inputs);

        // Calculate positions for odometry
        int sampleCount = inputs.odometryTimestamps.length; // All signals are sampled together
        odometryPositions = new SwerveModulePosition[sampleCount];
        for (int i = 0; i < sampleCount; i++) {
            Distance position = Meters.of(inputs.odometryDrivePositions[i].getRadians() * constants.WheelRadius);

            Rotation2d angle = inputs.odometrySteerPositions[i];
            odometryPositions[i] = new SwerveModulePosition(position, angle);
        }

        // Update alerts
        driveDisconnectedAlert.set(!inputs.driveConnected);
        steerDisconnectedAlert.set(!inputs.steerConnected);
        steerEncoderDisconnectedAlert.set(!inputs.steerEncoderConnected);
    }

    /** Runs the module with the specified setpoint state. Mutates the state to optimize it. */
    public void applySetpoint(SwerveModuleState state) {
        // Optimize velocity
        state.optimize(getAngle());
        state.cosineScale(inputs.steerPosition);

        // Apply setpoints
        io.setDriveVelocity(state.speedMetersPerSecond / constants.WheelRadius);
        io.setSteerPosition(state.angle);
    }

    /** Runs the module with the specified output while controlling to zero degrees. */
    public void runCharacterization(Voltage output) {
        io.setDriveOpenLoop(output.in(Volts));
        io.setSteerPosition(new Rotation2d());
    }

    /** Disables all outputs to motors. */
    public void stop() {
        io.setDriveOpenLoop(0.0);
        io.setSteerOpenLoop(0.0);
    }

    public Rotation2d getAngle() {
        return inputs.steerPosition;
    }

    public Distance getDrivePosition() {
        return Meters.of(inputs.drivePosition.in(Radians) * constants.WheelRadius);
    }

    public LinearVelocity getDriveVelocity() {
        return MetersPerSecond.of(inputs.driveAngularVelocity.in(RadiansPerSecond) * constants.WheelRadius);
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(getDrivePosition(), getAngle());
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(getDriveVelocity(), getAngle());
    }

    // Returns the module positions received this cycle
    public SwerveModulePosition[] getOdometryPositions() {
        return odometryPositions;
    }

    // Returns the timestamps of odometry samples received this cycle
    public double[] getOdometryTimestamps() {
        return inputs.odometryTimestamps;
    }
}
