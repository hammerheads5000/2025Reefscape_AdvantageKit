package frc.robot.subsystems.climber;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;

public interface ClimberIO {
    @AutoLog
    class ClimberIOInputs {
        public boolean motorConnected = false;
        public AngularVelocity velocity = RadiansPerSecond.zero();
        public Current torqueCurrent = Amps.zero();
    }

    default void updateInputs(ClimberIOInputs inputs) {}

    default void setOutput(Voltage output) {}

    default void stop() {}
}
