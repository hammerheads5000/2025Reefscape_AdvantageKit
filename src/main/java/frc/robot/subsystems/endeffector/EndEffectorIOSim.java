// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.endeffector;

import static edu.wpi.first.units.Units.Amps;

import frc.robot.Constants.EndEffectorConstants;

public class EndEffectorIOSim implements EndEffectorIO {
    public EndEffectorIOSim() {}

    @Override
    public void updateInputs(EndEffectorIOInputs inputs) {
        inputs.leftConnected = true;
        inputs.rightConnected = true;

        // Assume motors are running
        inputs.leftVelocity = EndEffectorConstants.MIN_VEL.times(2);
        inputs.rightVelocity = EndEffectorConstants.MIN_VEL.times(2);
        inputs.torqueCurrent = Amps.of(5);

        inputs.backLidar = false;
        inputs.frontLidar = false;
        inputs.intakeLidar = false;
    }
}
