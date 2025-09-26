package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.InchesPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {
    @AutoLog
    public static class ElevatorIOInputs {
        public boolean leadMotorConnected = false;
        public boolean followMotorConnected = false;
        public boolean encoderConnected = false;

        public Distance position = Inches.zero();
        public LinearVelocity velocity = InchesPerSecond.zero();
        public Voltage outputVoltage = Volts.zero();
        public Current outputCurrent = Amps.zero();

        public boolean manualOverride = false;
        public Distance setpointPos = Inches.zero();
        public LinearVelocity setpointVel = InchesPerSecond.zero();
    }

    /** Updates the set of loggable inputs. */
    public default void updateInputs(ElevatorIOInputs inputs) {}

    /** Run the elevator at the specified voltage. */
    public default void setOpenLoopOutput(Voltage output) {}

    public default void setGoal(Distance goal) {}

    public default void resetAtPosition() {}

    public default void setStage(Elevator.Stage stage) {}

    public default void setManualOverride(boolean override) {}

    /** Resets encoder position to current absolute position w/i a rotation */
    public default void resetEncoder() {}

    /** Zeroes encoder position to current absolute position (overwrites config) */
    public default void zeroEncoder() {}
}
