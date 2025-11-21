// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import edu.wpi.first.math.controller.PIDController;

/**
 * TunableController
 *
 * <p>Lightweight wrapper around WPILib's {@link PIDController} that configures the controller using runtime-tunable
 * values provided by a {@link frc.robot.util.TunableControlConstants}. This class centralizes PIDController setup
 * (gains, motion constraints, tolerances, and continuous input) and stores auxiliary state used by higher-level control
 * helpers.
 *
 * <p>Intended usage: pass a {@code TunableControlConstants} instance that exposes tunable parameters (kP, kI, kD,
 * tolerance, velTolerance, period, isContinuous, minInput, maxInput). The wrapper will construct and configure the
 * internal {@link PIDController} accordingly.
 */
public class TunablePIDController {
    private final TunableControlConstants params;
    private final PIDController pidController;

    public TunablePIDController(TunableControlConstants tunableParams) {
        this.params = tunableParams;

        pidController = new PIDController(
                tunableParams.kP.get(), tunableParams.kI.get(), tunableParams.kD.get(), tunableParams.period);

        pidController.setTolerance(tunableParams.tolerance.get(), tunableParams.velTolerance.get());

        if (tunableParams.isContinuous) {
            pidController.enableContinuousInput(tunableParams.minInput, tunableParams.maxInput);
        }
    }

    /**
     * Returns the TunableControlConstants used to configure this controller.
     *
     * @return The TunableControlConstants of this controller.
     */
    public TunableControlConstants getParams() {
        return params;
    }

    /** Updates the PIDController's parameters from the TunableControlConstants. */
    public void updateParams() {
        pidController.setP(params.kP.get());
        pidController.setI(params.kI.get());
        pidController.setD(params.kD.get());
        pidController.setTolerance(params.tolerance.get(), params.velTolerance.get());
        pidController.setIZone(params.iZone.get());
        pidController.setIntegratorRange(params.iMin.get(), params.iMax.get());
        pidController.setTolerance(params.tolerance.get(), params.velTolerance.get());
    }

    /**
     * Returns the accumulated error used in the integral calculation of this controller.
     *
     * @return The accumulated error of this controller.
     */
    public double getAccumulatedError() {
        return pidController.getAccumulatedError();
    }

    /**
     * Sets the goal for the PIDController.
     *
     * @param goal The desired goal.
     */
    public void setSetpoint(double goal) {
        pidController.setSetpoint(goal);
        updateParams();
    }

    /**
     * Returns the current goal of the PIDController.
     *
     * @return The current goal.
     */
    public double getSetpoint() {
        return pidController.getSetpoint();
    }

    /**
     * Returns true if the error is within the tolerance of the goal. The error tolerance defaults to 0.05, and the
     * error derivative tolerance defaults to ∞.
     *
     * <p>This will return false until at least one input value has been computed.
     *
     * @return Whether the error is within the acceptable bounds.
     */
    public boolean atSetpoint() {
        return pidController.atSetpoint();
    }

    /**
     * Returns the difference between the goal and the measurement.
     *
     * @return The error.
     */
    public double getPositionError() {
        return pidController.getError();
    }

    /**
     * Returns the error derivative.
     *
     * @return The error derivative.
     */
    public double getVelocityError() {
        return pidController.getErrorDerivative();
    }

    /**
     * Returns the next output of the PID controller.
     *
     * @param measurement The current measurement of the process variable.
     * @param goal The new goal of the controller.
     * @return The next controller output.
     */
    public double calculate(double measurement, double goal) {
        return pidController.calculate(measurement, goal);
    }

    /**
     * Returns the next output of the PID controller.
     *
     * @param measurement The current measurement of the process variable.
     * @return The next controller output.
     */
    public double calculate(double measurement) {
        return pidController.calculate(measurement);
    }

    /** Resets the previous error and the integral term. */
    public void reset() {
        pidController.reset();
    }
}
