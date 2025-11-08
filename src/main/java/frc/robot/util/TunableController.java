// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import edu.wpi.first.math.MathUtil;

/** Add your docs here. */
public class TunableController {
    // The error range where "integral" control applies
    private double iZone = Double.POSITIVE_INFINITY;

    // The period (in seconds) of the loop that calls the controller
    private final double PERIOD;

    private double maxIntegral = 1.0;

    private double minIntegral = -1.0;

    private double maxInput;

    private double minInput;

    // Do the endpoints wrap around? e.g. Absolute encoder
    private boolean isContinuous;

    // The error at the time of the most recent call to calculate()
    private double error;
    private double errorDerivative;

    // The error at the time of the second-most-recent call to calculate() (used to
    // compute velocity)
    private double prevError;

    // The sum of the errors for use in the integral calc
    private double totalError;

    // The error that is considered at setpoint.
    private double errorDerivativeTolerance = Double.POSITIVE_INFINITY;

    private double setpoint;
    private double measurement;

    private boolean hasMeasurement;
    private boolean hasSetpoint;

    private LoggedTunableNumber kP;
    private LoggedTunableNumber kI;
    private LoggedTunableNumber kD;
    private LoggedTunableNumber tolerance;

    /**
     * Allocates a PIDController with the given constants for kP, kI, and kD.
     *
     * @param kP     The proportional coefficient.
     * @param kI     The integral coefficient.
     * @param kD     The derivative coefficient.
     * @param period The period between controller updates in seconds.
     * @throws IllegalArgumentException if kP &lt; 0
     * @throws IllegalArgumentException if kI &lt; 0
     * @throws IllegalArgumentException if kD &lt; 0
     * @throws IllegalArgumentException if period &lt;= 0
     */
    @SuppressWarnings("this-escape")
    public TunableController(String key, ControlConstants params) {
        if (params.kP < 0.0) {
            throw new IllegalArgumentException("kP must be a non-negative number!");
        }
        if (params.kI < 0.0) {
            throw new IllegalArgumentException("kI must be a non-negative number!");
        }
        if (params.kD < 0.0) {
            throw new IllegalArgumentException("kD must be a non-negative number!");
        }
        if (params.period <= 0.0) {
            throw new IllegalArgumentException("Controller period must be a positive number!");
        }

        this.kP = new LoggedTunableNumber(key+"/kP", params.kP);
        this.kI = new LoggedTunableNumber(key+"/kI", params.kI);
        this.kD = new LoggedTunableNumber(key+"/kD", params.kD);
        tolerance = new LoggedTunableNumber(key+"/tolerance", params.tolerance);

        PERIOD = params.period;
        iZone = params.iZone;
        maxIntegral = params.iMax;
        minIntegral = params.iMin;
    }

    /**
     * Get the Proportional coefficient.
     *
     * @return proportional coefficient
     */
    public double getP() {
        return kP.get();
    }

    /**
     * Get the Integral coefficient.
     *
     * @return integral coefficient
     */
    public double getI() {
        return kI.get();
    }

    /**
     * Get the Differential coefficient.
     *
     * @return differential coefficient
     */
    public double getD() {
        return kD.get();
    }

    /**
     * Get the IZone range.
     *
     * @return Maximum magnitude of error to allow integral control.
     */
    public double getIZone() {
        return iZone;
    }

    /**
     * Returns the period of this controller.
     *
     * @return the period of the controller.
     */
    public double getPeriod() {
        return PERIOD;
    }

    /**
     * Returns the error tolerance of this controller. Defaults to 0.05.
     *
     * @return the error tolerance of the controller.
     */
    public double gettolerance() {
        return tolerance.get();
    }

    /**
     * Returns the error derivative tolerance of this controller. Defaults to ∞.
     *
     * @return the error derivative tolerance of the controller.
     */
    public double getErrorDerivativeTolerance() {
        return errorDerivativeTolerance;
    }

    /**
     * Returns the accumulated error used in the integral calculation of this
     * controller.
     *
     * @return The accumulated error of this controller.
     */
    public double getAccumulatedError() {
        return totalError;
    }

    /**
     * Sets the setpoint for the PIDController.
     *
     * @param setpoint The desired setpoint.
     */
    public void setSetpoint(double setpoint) {
        this.setpoint = setpoint;
        hasSetpoint = true;

        if (isContinuous) {
            double errorBound = (maxInput - minInput) / 2.0;
            error = MathUtil.inputModulus(setpoint - measurement, -errorBound, errorBound);
        } else {
            error = setpoint - measurement;
        }

        errorDerivative = (error - prevError) / PERIOD;
    }

    /**
     * Returns the current setpoint of the PIDController.
     *
     * @return The current setpoint.
     */
    public double getSetpoint() {
        return setpoint;
    }

    /**
     * Returns true if the error is within the tolerance of the setpoint. The error
     * tolerance defaults
     * to 0.05, and the error derivative tolerance defaults to ∞.
     *
     * <p>
     * This will return false until at least one input value has been computed.
     *
     * @return Whether the error is within the acceptable bounds.
     */
    public boolean atSetpoint() {
        return hasMeasurement
                && hasSetpoint
                && Math.abs(error) < tolerance.get()
                && Math.abs(errorDerivative) < errorDerivativeTolerance;
    }

    /**
     * Enables continuous input.
     *
     * <p>
     * Rather then using the max and min input range as constraints, it considers
     * them to be the
     * same point and automatically calculates the shortest route to the setpoint.
     *
     * @param minimumInput The minimum value expected from the input.
     * @param maximumInput The maximum value expected from the input.
     */
    public void enableContinuousInput(double minimumInput, double maximumInput) {
        isContinuous = true;
        minInput = minimumInput;
        maxInput = maximumInput;
    }

    /** Disables continuous input. */
    public void disableContinuousInput() {
        isContinuous = false;
    }

    /**
     * Returns true if continuous input is enabled.
     *
     * @return True if continuous input is enabled.
     */
    public boolean isContinuousInputEnabled() {
        return isContinuous;
    }

    /**
     * Sets the minimum and maximum contributions of the integral term.
     *
     * <p>
     * The internal integrator is clamped so that the integral term's contribution
     * to the output
     * stays between minimumIntegral and maximumIntegral. This prevents integral
     * windup.
     *
     * @param minimumIntegral The minimum contribution of the integral term.
     * @param maximumIntegral The maximum contribution of the integral term.
     */
    public void setIntegratorRange(double minimumIntegral, double maximumIntegral) {
        minIntegral = minimumIntegral;
        maxIntegral = maximumIntegral;
    }

    /**
     * Returns the difference between the setpoint and the measurement.
     *
     * @return The error.
     * @deprecated Use getError() instead.
     */
    @Deprecated(forRemoval = true, since = "2025")
    public double getPositionError() {
        return error;
    }

    /**
     * Returns the velocity error.
     *
     * @return The velocity error.
     * @deprecated Use getErrorDerivative() instead.
     */
    @Deprecated(forRemoval = true, since = "2025")
    public double getVelocityError() {
        return errorDerivative;
    }

    /**
     * Returns the difference between the setpoint and the measurement.
     *
     * @return The error.
     */
    public double getError() {
        return error;
    }

    /**
     * Returns the error derivative.
     *
     * @return The error derivative.
     */
    public double getErrorDerivative() {
        return errorDerivative;
    }

    /**
     * Returns the next output of the PID controller.
     *
     * @param measurement The current measurement of the process variable.
     * @param setpoint    The new setpoint of the controller.
     * @return The next controller output.
     */
    public double calculate(double measurement, double setpoint) {
        this.setpoint = setpoint;
        hasSetpoint = true;
        return calculate(measurement);
    }

    /**
     * Returns the next output of the PID controller.
     *
     * @param measurement The current measurement of the process variable.
     * @return The next controller output.
     */
    public double calculate(double measurement) {
        this.measurement = measurement;
        prevError = error;
        hasMeasurement = true;

        if (isContinuous) {
            double errorBound = (maxInput - minInput) / 2.0;
            error = MathUtil.inputModulus(setpoint - measurement, -errorBound, errorBound);
        } else {
            error = setpoint - measurement;
        }

        errorDerivative = (error - prevError) / PERIOD;

        // If the absolute value of the position error is greater than IZone, reset the
        // total error
        if (Math.abs(error) > iZone) {
            totalError = 0;
        } else if (kI.get() != 0) {
            totalError = MathUtil.clamp(
                    totalError + error * PERIOD,
                    minIntegral / kI.get(),
                    maxIntegral / kI.get());
        }

        return kP.get() * error + kI.get() * totalError + kD.get() * errorDerivative;
    }

    /** Resets the previous error and the integral term. */
    public void reset() {
        error = 0;
        prevError = 0;
        totalError = 0;
        errorDerivative = 0;
        hasMeasurement = false;
    }
}
