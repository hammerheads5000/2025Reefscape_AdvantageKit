// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import frc.robot.Constants.ElevatorConstants;

public class ElevatorIOSim implements ElevatorIO {
    private final DCMotor gearbox = DCMotor.getKrakenX60Foc(2);

    private final ElevatorSim sim = new ElevatorSim(
            gearbox,
            ElevatorConstants.GEAR_RATIO,
            ElevatorConstants.CARRIAGE_MASS.in(Kilograms),
            ElevatorConstants.DRUM_RADIUS.in(Meters) * 3, // to simulate cascading
            ElevatorConstants.MIN_HEIGHT.in(Meters),
            ElevatorConstants.MAX_HEIGHT.in(Meters),
            true,
            ElevatorConstants.MIN_HEIGHT.in(Meters),
            0.001,
            0.0);

    private final ProfiledPIDController controller =
            new ProfiledPIDController(0.001, 0, 0, new TrapezoidProfile.Constraints(100, 100));
    private final ElevatorFeedforward feedforward = new ElevatorFeedforward(0.09, 0.01, 0.118, 0.0);

    private Voltage manualVoltage = Volts.zero();
    private boolean manualOverride = false;

    public ElevatorIOSim() {}

    @Override
    public void updateInputs(ElevatorIOInputs inputs) {
        double voltage;
        if (inputs.manualOverride) {
            voltage = manualVoltage.in(Volts);
        } else {
            voltage = controller.calculate(metersToRadians(sim.getPositionMeters()));
            voltage += feedforward.calculate(controller.getSetpoint().velocity);
        }
        sim.setInput(voltage);
        sim.update(0.02);
        RoboRioSim.setVInVoltage(BatterySim.calculateDefaultBatteryLoadedVoltage(sim.getCurrentDrawAmps()));

        inputs.position = Radians.of(metersToRadians(sim.getPositionMeters()));
        inputs.velocity = RadiansPerSecond.of(metersToRadians(sim.getVelocityMetersPerSecond()));

        inputs.leadMotorConnected = true;
        inputs.followMotorConnected = true;
        inputs.encoderConnected = true;

        inputs.outputCurrent = Amps.of(sim.getCurrentDrawAmps());
        inputs.outputVoltage = Volts.of(gearbox.getVoltage(
                gearbox.getTorque(sim.getCurrentDrawAmps()), metersToRadians(sim.getVelocityMetersPerSecond())));

        inputs.setpointPos = Radians.of(controller.getSetpoint().position);
        inputs.setpointVel = RadiansPerSecond.of(controller.getSetpoint().velocity);
        inputs.manualOverride = manualOverride;
    }

    @SuppressWarnings("unused")
    private double radiansToMeters(double radians) {
        return ElevatorConstants.MIN_HEIGHT.in(Meters) + radians * ElevatorConstants.DRUM_RADIUS.in(Meters) * 3;
    }

    private double metersToRadians(double meters) {
        return (meters - ElevatorConstants.MIN_HEIGHT.in(Meters)) / ElevatorConstants.DRUM_RADIUS.in(Meters) / 3;
    }

    @Override
    public void setOpenLoopOutput(Voltage output) {
        manualVoltage = output;
    }

    @Override
    public void setGoal(Angle goal) {
        manualOverride = false;
        controller.setGoal(goal.in(Radians));
    }

    @Override
    public void resetAtPosition() {
        manualVoltage = Volts.of(feedforward.getKg());
        controller.reset(metersToRadians(sim.getPositionMeters()), 0);
    }

    @Override
    public void setPIDConstants(double kP, double kI, double kD, double kV, double kA, double kS, double kG) {
        controller.setPID(kP, kI, kD);
        feedforward.setKa(kA);
        feedforward.setKv(kV);
        feedforward.setKs(kS);
        feedforward.setKg(kG);
    }

    @Override
    public void setManualOverride(boolean override) {
        manualOverride = override;
    }
}
