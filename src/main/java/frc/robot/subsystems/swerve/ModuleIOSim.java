// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.subsystems.swerve;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

/**
 * Physics sim implementation of module IO. The sim models are configured using a set of module constants from Phoenix.
 * Simulation is always based on voltage control.
 */
public class ModuleIOSim implements ModuleIO {
    // TunerConstants doesn't support separate sim constants, so they are declared
    // locally
    private static final double DRIVE_KP = 0.05;
    private static final double DRIVE_KD = 0.0;
    private static final double DRIVE_KS = 0.0;
    private static final double DRIVE_KV_ROT = 0.91035; // Same units as TunerConstants: (volt * secs) / rotation
    private static final double DRIVE_KV = 1.0 / Units.rotationsToRadians(1.0 / DRIVE_KV_ROT);
    private static final double TURN_KP = 8.0;
    private static final double TURN_KD = 0.0;
    private static final DCMotor DRIVE_GEARBOX = DCMotor.getKrakenX60Foc(1);
    private static final DCMotor TURN_GEARBOX = DCMotor.getKrakenX60Foc(1);

    private final DCMotorSim driveSim;
    private final DCMotorSim turnSim;

    private boolean driveClosedLoop = false;
    private boolean turnClosedLoop = false;
    private PIDController driveController = new PIDController(DRIVE_KP, 0, DRIVE_KD);
    private PIDController steerController = new PIDController(TURN_KP, 0, TURN_KD);
    private Voltage driveFFVolts = Volts.zero();
    private Voltage driveAppliedVolts = Volts.zero();
    private Voltage steerAppliedVolts = Volts.zero();

    public ModuleIOSim(
            SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> constants) {
        // Create drive and turn sim models
        driveSim = new DCMotorSim(
                LinearSystemId.createDCMotorSystem(
                        DRIVE_GEARBOX, constants.DriveInertia, constants.DriveMotorGearRatio),
                DRIVE_GEARBOX);
        turnSim = new DCMotorSim(
                LinearSystemId.createDCMotorSystem(TURN_GEARBOX, constants.SteerInertia, constants.SteerMotorGearRatio),
                TURN_GEARBOX);

        // Enable wrapping for turn PID
        steerController.enableContinuousInput(-Math.PI, Math.PI);
    }

    @Override
    public void updateInputs(ModuleIOInputs inputs) {
        // Run closed-loop control
        if (driveClosedLoop) {
            driveAppliedVolts =
                    driveFFVolts.plus(Volts.of(driveController.calculate(driveSim.getAngularVelocityRadPerSec())));
        } else {
            driveController.reset();
        }
        if (turnClosedLoop) {
            steerAppliedVolts = Volts.of(steerController.calculate(turnSim.getAngularPositionRad()));
        } else {
            steerController.reset();
        }

        // Update simulation state
        driveSim.setInputVoltage(MathUtil.clamp(driveAppliedVolts.in(Volts), -12.0, 12.0));
        turnSim.setInputVoltage(MathUtil.clamp(steerAppliedVolts.in(Volts), -12.0, 12.0));
        driveSim.update(0.02);
        turnSim.update(0.02);

        // Update drive inputs
        inputs.driveConnected = true;
        inputs.drivePosition = driveSim.getAngularPosition();
        inputs.driveAngularVelocity = driveSim.getAngularVelocity();
        inputs.driveAppliedVolts = driveAppliedVolts;
        inputs.driveCurrent = Amps.of(Math.abs(driveSim.getCurrentDrawAmps()));

        // Update turn inputs
        inputs.steerConnected = true;
        inputs.steerEncoderConnected = true;
        inputs.steerPosition = new Rotation2d(turnSim.getAngularPositionRad());
        inputs.steerVelocity = turnSim.getAngularVelocity();
        inputs.steerAppliedVolts = steerAppliedVolts;
        inputs.steerCurrent = Amps.of(Math.abs(turnSim.getCurrentDrawAmps()));

        // Update odometry inputs (50Hz because high-frequency odometry in sim doesn't
        // matter)
        inputs.odometryTimestamps = new double[] {Timer.getFPGATimestamp()};
        inputs.odometryDrivePositions = new Rotation2d[] {new Rotation2d(inputs.drivePosition)};
        inputs.odometrySteerPositions = new Rotation2d[] {inputs.steerPosition};
    }

    @Override
    public void setDriveOpenLoop(double output) {
        driveClosedLoop = false;
        driveAppliedVolts = Volts.of(output);
    }

    @Override
    public void setSteerOpenLoop(double output) {
        turnClosedLoop = false;
        steerAppliedVolts = Volts.of(output);
    }

    @Override
    public void setDriveVelocity(double velocityRadPerSec) {
        driveClosedLoop = true;
        driveFFVolts = Volts.of(DRIVE_KS * Math.signum(velocityRadPerSec) + DRIVE_KV * velocityRadPerSec);
        driveController.setSetpoint(velocityRadPerSec);
    }

    @Override
    public void setSteerPosition(Rotation2d rotation) {
        turnClosedLoop = true;
        steerController.setSetpoint(rotation.getRadians());
    }
}
