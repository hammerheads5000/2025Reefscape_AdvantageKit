// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.algaemanipulator;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.Voltage;

public class AlgaeManipulatorIOSim implements AlgaeManipulatorIO {
    private Voltage speed = Volts.zero();

    public AlgaeManipulatorIOSim() {}

    @Override
    public void updateInputs(AlgaeManipulatorIOInputs inputs) {
        inputs.motorConnected = true;

        // Assume motors are running
        inputs.velocity = RotationsPerSecond.of(0.5).times(2);
        inputs.appliedVolts = speed;
        inputs.torqueCurrent = Amps.of(5);
    }

    @Override
    public void setSpeed(Voltage speed) {
        this.speed = speed;
    }

    @Override
    public void stop() {
        this.speed = Volts.zero();
    }
}
