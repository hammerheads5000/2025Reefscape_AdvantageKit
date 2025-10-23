package frc.robot.subsystems.climber;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;

public interface ClimberIO {
    @AutoLog
    class ClimberIOInputs {
        public boolean climbMotorConnected = false;
        public AngularVelocity climbVelocity = RadiansPerSecond.zero();
        public Voltage climbAppliedVolts = Volts.zero();
        public Current climbTorqueCurrent = Amps.zero();

        public boolean grabMotorConnected = false;
        public AngularVelocity grabVelocity = RadiansPerSecond.zero();
        public Voltage grabAppliedVolts = Volts.zero();

        public boolean encoderConnected = false;
        public Angle pos = Radians.zero();

        public boolean cageDetected = false;
    }

    default void updateInputs(ClimberIOInputs inputs) {}

    default void setClimberOutput(Voltage output) {}

    default void setGrabOutput(Voltage output) {}

    default void stopClimb() {}

    default void stopGrab() {}
}
