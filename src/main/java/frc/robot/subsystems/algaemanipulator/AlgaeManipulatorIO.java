package frc.robot.subsystems.algaemanipulator;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;

public interface AlgaeManipulatorIO {
    @AutoLog
    class AlgaeManipulatorIOInputs {
        public boolean motorConnected = false;
        public AngularVelocity velocity = RadiansPerSecond.zero();
        public Current torqueCurrent = Amps.zero();
        public Voltage appliedVolts = Volts.zero();

        public boolean lidarSeesAlgae = false;
    }

    default void updateInputs(AlgaeManipulatorIOInputs inputs) {}

    default void setSpeed(Voltage speed) {}

    default void stop() {}
}
