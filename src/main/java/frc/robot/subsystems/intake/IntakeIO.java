// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
    @AutoLog
    class IntakeIOInputs {
        public boolean intakeMotorConnected = false;
        public boolean deployMotorConnected = false;
        public boolean alignMotorConnected = false;

        public AngularVelocity intakeVelocity = RadiansPerSecond.zero();
        public AngularVelocity alignVelocity = RadiansPerSecond.zero();
        public AngularVelocity deployVelocity = RadiansPerSecond.zero();

        public Angle setpointPos = Radians.zero();
        public AngularVelocity setpointVel = RadiansPerSecond.zero();

        public Current intakeCurrent = Amps.zero();
        public Current alignCurrent = Amps.zero();
        public Current deployTorqueCurrent = Amps.zero();

        public boolean alignLidar = false;
        public boolean coralDetected = false; // beam break sensor

        public Angle position = Radians.zero();
    }

    default void updateInputs(IntakeIOInputs inputs) {}

    default void setGoal(Angle angle) {}

    default void setIntakeSpeed(Voltage speed) {}

    default void setAlignSpeed(Voltage speed) {}

    default void stopDeploy() {}

    default void setToCoast(boolean shouldCoast) {}
}
