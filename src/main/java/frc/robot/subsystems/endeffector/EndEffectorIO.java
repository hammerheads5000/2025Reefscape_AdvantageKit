package frc.robot.subsystems.endeffector;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;

public interface EndEffectorIO {
    @AutoLog
    class EndEffectorIOInputs {
        public boolean leftConnected = false;
        public boolean rightConnected = false;
        public AngularVelocity leftVelocity = RadiansPerSecond.zero();
        public AngularVelocity rightVelocity = RadiansPerSecond.zero();
        public Current torqueCurrent = Amps.zero();

        public boolean frontLidar = false;
        public boolean backLidar = false;
        public boolean intakeLidar = false;
    }

    default void updateInputs(EndEffectorIOInputs inputs) {}

    default void setSpeeds(Voltage left, Voltage right) {}

    default void stop() {}
}
