package frc.robot.subsystems.coraldetection;

import edu.wpi.first.math.geometry.Translation2d;
import org.littletonrobotics.junction.AutoLog;

public interface CoralDetectionIO {
    @AutoLog
    public static class CoralDetectionIOInputs {
        public boolean connected = false;

        // list of coral angles: (yaw, pitch) in degrees
        public Translation2d[] corals = new Translation2d[0];
    }

    public default void updateInputs(CoralDetectionIOInputs inputs) {}
}
