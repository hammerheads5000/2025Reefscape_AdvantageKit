// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;

/**
 * TunableController
 *
 * <p>Lightweight wrapper around WPILib's {@link ProfiledPIDController} that configures the controller using
 * runtime-tunable values provided by a {@link frc.robot.util.TunableControlConstants}. This class centralizes
 * ProfiledPIDController setup (gains, motion constraints, tolerances, and continuous input) and stores auxiliary state
 * used by higher-level control helpers.
 *
 * <p>Intended usage: pass a {@code TunableControlConstants} instance that exposes tunable parameters (kP, kI, kD,
 * maxVel, maxAcc, tolerance, velTolerance, period, isContinuous, minInput, maxInput). The wrapper will construct and
 * configure the internal {@link ProfiledPIDController} accordingly.
 */
public class TunableProfiledController {
    private final ProfiledPIDController profiledPIDController;
    private final TunableControlConstants params;

    private double previousVelocity = 0;

    public TunableProfiledController(TunableControlConstants tunableParams) {
        this.params = tunableParams;

        if (!tunableParams.profiled) {
            throw new IllegalArgumentException(
                    "TunableControlConstants must be profiled to use TunableProfiledController");
        }

        profiledPIDController = new ProfiledPIDController(
                tunableParams.kP.get(),
                tunableParams.kI.get(),
                tunableParams.kD.get(),
                new TrapezoidProfile.Constraints(tunableParams.maxVel.get(), tunableParams.maxAcc.get()),
                tunableParams.period);

        profiledPIDController.setTolerance(tunableParams.tolerance.get(), tunableParams.velTolerance.get());

        if (tunableParams.isContinuous) {
            profiledPIDController.enableContinuousInput(tunableParams.minInput, tunableParams.maxInput);
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

    /** Updates the controller's parameters based on the current values in the TunableControlConstants. */
    public void updateParams() {
        profiledPIDController.setP(params.kP.get());
        profiledPIDController.setI(params.kI.get());
        profiledPIDController.setD(params.kD.get());

        profiledPIDController.setConstraints(
                new TrapezoidProfile.Constraints(params.maxVel.get(), params.maxAcc.get()));
        profiledPIDController.setIZone(params.iZone.get());
        profiledPIDController.setIntegratorRange(params.iMin.get(), params.iMax.get());
        profiledPIDController.setTolerance(params.tolerance.get(), params.velTolerance.get());
    }

    /**
     * Returns the accumulated error used in the integral calculation of this controller.
     *
     * @return The accumulated error of this controller.
     */
    public double getAccumulatedError() {
        return profiledPIDController.getAccumulatedError();
    }

    /**
     * Sets the goal for the ProfiledPIDController.
     *
     * @param goal The desired goal.
     */
    public void setGoal(double goal) {
        profiledPIDController.setGoal(goal);
        previousVelocity = profiledPIDController.getSetpoint().velocity;
        updateParams();
    }

    /**
     * Returns the current goal of the ProfiledPIDController.
     *
     * @return The current goal.
     */
    public double getGoal() {
        return profiledPIDController.getGoal().position;
    }

    public State getSetpoint() {
        return profiledPIDController.getSetpoint();
    }

    /**
     * Returns true if the error is within the tolerance of the goal. The error tolerance defaults to 0.05, and the
     * error derivative tolerance defaults to ∞.
     *
     * <p>This will return false until at least one input value has been computed.
     *
     * @return Whether the error is within the acceptable bounds.
     */
    public boolean atGoal() {
        return profiledPIDController.atGoal();
    }

    /**
     * Returns the difference between the goal and the measurement.
     *
     * @return The error.
     */
    public double getPositionError() {
        return profiledPIDController.getPositionError();
    }

    /**
     * Returns the error derivative.
     *
     * @return The error derivative.
     */
    public double getVelocityError() {
        return profiledPIDController.getVelocityError();
    }

    public double calculateFeedforward() {
        State setpoint = profiledPIDController.getSetpoint();
        double accel = (setpoint.velocity - previousVelocity) / profiledPIDController.getPeriod();
        previousVelocity = setpoint.velocity;

        return params.kS.get() * Math.signum(setpoint.velocity)
                + params.kG.get()
                + params.kV.get() * setpoint.velocity
                + params.kA.get() * accel;
    }

    /**
     * Returns the next output of the PID controller.
     *
     * @param measurement The current measurement of the process variable.
     * @param goal The new goal of the controller.
     * @return The next controller output.
     */
    public double calculate(double measurement, double goal) {
        return profiledPIDController.calculate(measurement, goal) + calculateFeedforward();
    }

    /**
     * Returns the next output of the PID controller.
     *
     * @param measurement The current measurement of the process variable.
     * @return The next controller output.
     */
    public double calculate(double measurement) {
        return profiledPIDController.calculate(measurement) + calculateFeedforward();
    }

    /** Resets the previous error and the integral term. */
    public void reset(double measuredPos, double measuredVel) {
        profiledPIDController.reset(measuredPos, measuredVel);
        previousVelocity = measuredVel;
    }

    /** Resets the previous error and the integral term. */
    public void reset(double measuredPos) {
        reset(measuredPos, 0);
    }
}
