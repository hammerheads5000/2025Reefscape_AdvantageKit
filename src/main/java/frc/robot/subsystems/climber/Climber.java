// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climber;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;

public class Climber extends SubsystemBase {
    private final ClimberIO io;
    private final ClimberIOInputsAutoLogged inputs = new ClimberIOInputsAutoLogged();

    public Climber(ClimberIO io) {
        this.io = io;

        SmartDashboard.putData("Climb", climbCommand());
        SmartDashboard.putData("Unclimb", reverseCommand());
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Climber", inputs);
    }

    public Command climbCommand() {
        return this.startEnd(() -> io.setOutput(ClimberConstants.CLIMB_SPEED), io::stop);
    }

    public Command reverseCommand() {
        return this.startEnd(() -> io.setOutput(ClimberConstants.RELEASE_SPEED), io::stop);
    }
}
