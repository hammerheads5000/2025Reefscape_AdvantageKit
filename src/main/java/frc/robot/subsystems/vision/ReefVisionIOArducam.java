// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import static edu.wpi.first.units.Units.Rotations;

import com.ctre.phoenix6.hardware.CANdi;
import frc.robot.Constants;
import frc.robot.Constants.VisionConstants;

public class ReefVisionIOArducam implements ReefVisionIO {
    CANdi candi;

    public ReefVisionIOArducam() {
        candi = new CANdi(VisionConstants.REEF_VISION_CANDI_ID, Constants.CAN_FD_BUS);
    }

    @Override
    public void updateInputs(ReefVisionIOInputs inputs) {
        double dutyCycleAngle = candi.getPWM1Position().getValue().in(Rotations);
        dutyCycleAngle = (dutyCycleAngle - 0.5) * 2; // center around 0
        double dutyCycleDistance = candi.getPWM2Position().getValue().in(Rotations);

        inputs.angle = VisionConstants.HORIZONTAL_FOV.times(dutyCycleAngle / 2);
        inputs.distance = VisionConstants.MAX_DISTANCE.times((dutyCycleDistance + 1) / 2);
    }
}
