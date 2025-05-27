// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climber;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Voltage;

/** Add your docs here. */
public class ClimberIOSim implements ClimberIO {
    private final DCMotor motor = DCMotor.getKrakenX60(1);
    private Voltage output = Volts.zero();

    public ClimberIOSim() {}

    @Override
    public void updateInputs(ClimberIOInputs inputs) {
        inputs.climbMotorConnected = true;
        inputs.climbVelocity = RadiansPerSecond.of(motor.getSpeed(3, output.in(Volts)));
        inputs.climbTorqueCurrent = Amps.of(motor.getCurrent(3));
        inputs.climbAppliedVolts = output;

        inputs.grabMotorConnected = true;
        inputs.grabVelocity = RadiansPerSecond.zero();
        inputs.grabAppliedVolts = Volts.zero();
        inputs.cageDetected = false;

        inputs.encoderConnected = true;
        inputs.pos = Radians.zero();
    }

    @Override
    public void setClimberOutput(Voltage output) {
        this.output = output;
    }

    @Override
    public void stopClimb() {
        this.output = Volts.zero();
    }
}
