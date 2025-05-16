// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.subsystems.elevator.Elevator.Stage;
import frc.robot.util.LoggedTunableNumber;

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

    // Tunable parameters (radians)
    private final LoggedTunableNumber kP =
            new LoggedTunableNumber("Elevator/Sim_kP", ElevatorConstants.STAGE3_GAINS.kP);
    private final LoggedTunableNumber kI =
            new LoggedTunableNumber("Elevator/Sim_kI", ElevatorConstants.STAGE3_GAINS.kI);
    private final LoggedTunableNumber kD =
            new LoggedTunableNumber("Elevator/Sim_kD", ElevatorConstants.STAGE3_GAINS.kD);
    private final LoggedTunableNumber kA =
            new LoggedTunableNumber("Elevator/Sim_kA", ElevatorConstants.STAGE3_GAINS.kA);
    private final LoggedTunableNumber kV =
            new LoggedTunableNumber("Elevator/Sim_kV", ElevatorConstants.STAGE3_GAINS.kV);
    private final LoggedTunableNumber kS =
            new LoggedTunableNumber("Elevator/Sim_kS", ElevatorConstants.STAGE3_GAINS.kS);
    private final LoggedTunableNumber kG =
            new LoggedTunableNumber("Elevator/Sim_kG", ElevatorConstants.STAGE3_GAINS.kG);

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

        inputs.position = Meters.of(sim.getPositionMeters());
        inputs.velocity = MetersPerSecond.of(sim.getVelocityMetersPerSecond());

        inputs.leadMotorConnected = true;
        inputs.followMotorConnected = true;
        inputs.encoderConnected = true;

        inputs.outputCurrent = Amps.of(sim.getCurrentDrawAmps());
        inputs.outputVoltage = Volts.of(gearbox.getVoltage(
                gearbox.getTorque(sim.getCurrentDrawAmps()), metersToRadians(sim.getVelocityMetersPerSecond())));

        inputs.setpointPos = Meters.of(controller.getSetpoint().position);
        inputs.setpointVel = MetersPerSecond.of(controller.getSetpoint().velocity);
        inputs.manualOverride = manualOverride;

        updateTunableParameters();
    }

    private void updateTunableParameters() {
        // Update tunable parameters
        if (kP.hasChanged(hashCode())
                || kI.hasChanged(hashCode())
                || kD.hasChanged(hashCode())
                || kA.hasChanged(hashCode())
                || kV.hasChanged(hashCode())
                || kS.hasChanged(hashCode())
                || kG.hasChanged(hashCode())) {
            setPIDConstants(kP.get(), kI.get(), kD.get(), kV.get(), kA.get(), kS.get(), kG.get());
        }
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
    public void setGoal(Distance goal) {
        manualOverride = false;
        controller.setGoal(goal.in(Meters));
    }

    @Override
    public void resetAtPosition() {
        manualVoltage = Volts.of(feedforward.getKg());
        controller.reset(metersToRadians(sim.getPositionMeters()), 0);
    }

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

    @Override
    public void setStage(Stage stage) {
        System.out.println(stage);
    }
}
