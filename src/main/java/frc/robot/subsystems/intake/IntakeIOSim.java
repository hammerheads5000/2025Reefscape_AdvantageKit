// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants.IntakeConstants;

/** Add your docs here. */
public class IntakeIOSim implements IntakeIO {
    private final ProfiledPIDController deployController = IntakeConstants.DEPLOY_PID_SIM.getProfiledPIDController();
    private AngularVelocity intakeVelocity = RadiansPerSecond.of(0);
    private AngularVelocity alignVelocity = RadiansPerSecond.of(0);
    private AngularVelocity deployVelocity = RadiansPerSecond.of(0);

    public IntakeIOSim() {}

    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        deployController.calculate(deployController.getSetpoint().position);
        deployVelocity = RadiansPerSecond.of(deployController.getSetpoint().velocity);

        inputs.intakeMotorConnected = true;
        inputs.alignMotorConnected = true;
        inputs.deployMotorConnected = true;

        inputs.intakeVelocity = intakeVelocity;
        inputs.alignVelocity = alignVelocity;
        inputs.deployVelocity = deployVelocity;

        inputs.position = Radians.of(deployController.getSetpoint().position);

        inputs.setpointPos = Radians.of(deployController.getSetpoint().position);
        inputs.setpointVel = deployVelocity;
    }

    @Override
    public void setGoal(Angle angle) {
        deployController.setGoal(angle.in(Radians));
    }

    @Override
    public void setIntakeSpeed(Voltage speed) {
        intakeVelocity = RadiansPerSecond.of(speed.in(Volts) / 6);
    }

    @Override
    public void setAlignSpeed(Voltage speed) {
        alignVelocity = RadiansPerSecond.of(speed.in(Volts) / 6);
    }
}
