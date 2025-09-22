// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import org.littletonrobotics.junction.AutoLog;

public interface ReefVisionIO {
    @AutoLog
    public static class ReefVisionIOInputs {
        public Angle angle = Radians.zero();
        public Distance distance = Meters.zero();
    }

    public default void updateInputs(ReefVisionIOInputs inputs) {}
}
