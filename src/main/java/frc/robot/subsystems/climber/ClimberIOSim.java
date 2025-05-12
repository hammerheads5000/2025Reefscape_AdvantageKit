// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climber;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Voltage;

/** Add your docs here. */
public class ClimberIOSim implements ClimberIO {
    private final DCMotor motor = DCMotor.getKrakenX60(1);
    private Voltage output = Volts.zero();

    public ClimberIOSim() {
    }

    @Override
    public void updateInputs(ClimberIOInputs inputs) {
        inputs.motorConnected = true;
        inputs.velocity = RadiansPerSecond.of(motor.getSpeed(3, output.in(Volts)));
        inputs.torqueCurrent = Amps.of(motor.getCurrent(3));
    }

    @Override
    public void setOutput(Voltage output) {
        this.output = output;
    }

    @Override
    public void stop() {
        this.output = Volts.zero();
    }
}
