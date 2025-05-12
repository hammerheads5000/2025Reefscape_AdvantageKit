package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {
    @AutoLog
    public static class ElevatorIOInputs {
        public boolean leadMotorConnected = false;
        public boolean followMotorConnected = false;
        public boolean encoderConnected = false;

        public Angle position = Radians.zero();
        public AngularVelocity velocity = RadiansPerSecond.zero();
        public Voltage outputVoltage = Volts.zero();
        public Current outputCurrent = Amps.zero();

        public boolean manualOverride = false;
        public Angle setpointPos = Radians.zero();
        public AngularVelocity setpointVel = RadiansPerSecond.zero();
    }

    /** Updates the set of loggable inputs. */
    public default void updateInputs(ElevatorIOInputs inputs) {}

    /** Run the elevator at the specified voltage. */
    public default void setOpenLoopOutput(Voltage output) {}

    public default void setGoal(Angle goal) {}

    public default void resetAtPosition() {}

    public default void setPIDConstants(double kP, double KI, double kD, double kV, double kA, double kS, double kG) {}

    public default void setManualOverride(boolean override) {}
}
