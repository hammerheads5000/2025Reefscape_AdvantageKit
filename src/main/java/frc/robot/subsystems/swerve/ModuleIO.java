// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.swerve;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;

/** Add your docs here. */
public interface ModuleIO {
    @AutoLog
    public static class ModuleIOInputs {
        public boolean driveConnected = false;
        public Angle drivePosition = Radians.zero();
        public AngularVelocity driveAngularVelocity = RadiansPerSecond.zero();
        public Voltage driveAppliedVolts = Volts.zero();
        public Current driveCurrent = Amps.zero();

        public boolean steerConnected = false;
        public boolean steerEncoderConnected = false;
        public Rotation2d steerPosition = new Rotation2d();
        public AngularVelocity steerVelocity = RadiansPerSecond.zero();
        public Voltage steerAppliedVolts = Volts.zero();
        public Current steerCurrent = Amps.zero();

        public double[] odometryTimestamps = new double[] {};
        public Rotation2d[] odometryDrivePositions = new Rotation2d[] {};
        public Rotation2d[] odometrySteerPositions = new Rotation2d[] {};
    }

    /** Updates the set of loggable inputs. */
    public default void updateInputs(ModuleIOInputs inputs) {}

    /** Run the drive motor at the specified open loop value. */
    public default void setDriveOpenLoop(double output) {}

    /** Run the turn motor at the specified open loop value. */
    public default void setSteerOpenLoop(double output) {}

    /** Run the drive motor at the specified velocity. */
    public default void setDriveVelocity(double velocityRadPerSec) {}

    /** Run the turn motor to the specified rotation. */
    public default void setSteerPosition(Rotation2d rotation) {}
}
