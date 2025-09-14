// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.coraldetection;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class CoralDetection extends SubsystemBase {
    private CoralDetectionIO io;
    private CoralDetectionIOInputsAutoLogged inputs;

    private Alert disconnectedAlert;

    /** Creates a new CoralDetection. */
    public CoralDetection(CoralDetectionIO io) {
        this.io = io;
        this.inputs = new CoralDetectionIOInputsAutoLogged();

        this.disconnectedAlert = new Alert("Coral Camera disconnected", AlertType.kWarning);
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("CoralDetection", inputs);

        if (!inputs.connected) {
            disconnectedAlert.set(true);
        } else {
            disconnectedAlert.set(false);
        }
    }

    public Translation2d getClosestCoral() {
        Translation2d closest = null;
        for (var coral : inputs.corals) {
            if (closest == null || coral.getY() < closest.getY()) {
                closest = coral;
            }
        }
        return closest;
    }
}
