// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.coraldetection;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.util.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;

/** Add your docs here. */
public class CoralDetectionIOSim implements CoralDetectionIO {
    private LoggedTunableNumber simulatedCoralX = new LoggedTunableNumber("CoralDetection/SimulatedCoralX", 2.0);
    private LoggedTunableNumber simulatedCoralY = new LoggedTunableNumber("CoralDetection/SimulatedCoralY", 2.0);
    private LoggedTunableNumber simulatedCoralTheta =
            new LoggedTunableNumber("CoralDetection/SimulatedCoralTheta", 0.0);
    private LoggedTunableNumber simulateCoral = new LoggedTunableNumber("CoralDetection/Simulate Coral (0,1)", 1);

    public CoralDetectionIOSim() {}

    @Override
    public void updateInputs(CoralDetectionIOInputs inputs) {
        inputs.connected = true;

        Translation2d[] corals2d = new Translation2d[0];
        Pose3d[] corals3d = new Pose3d[0];
        if (simulateCoral.get() == 1) {
            corals2d = new Translation2d[]{new Translation2d(simulatedCoralX.get(), simulatedCoralY.get())};

            corals3d = new Pose3d[]{
                new Pose3d(
                        corals2d[0].getX(), corals2d[0].getY(), 0.05, new Rotation3d(0, 0, simulatedCoralTheta.get()))
            };
        }
        inputs.corals = corals2d;
        Logger.recordOutput("CoralDetection/Corals3d", corals3d);
    }
}
