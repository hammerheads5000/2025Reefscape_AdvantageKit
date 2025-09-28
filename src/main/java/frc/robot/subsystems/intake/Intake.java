// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.IntakeConstants;
import java.util.function.Supplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {
    IntakeIO io;
    IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

    Trigger alignerHasPiece = new Trigger(() -> inputs.alignLidar).debounce(0.1);

    @AutoLogOutput
    Trigger deployedTrigger = new Trigger(this::isDeployed).debounce(0.1);

    @AutoLogOutput
    Trigger stowedTrigger = new Trigger(this::isStowed).debounce(0.1);

    @AutoLogOutput
    public Trigger coralDetectedTrigger = new Trigger(this::rawCoralDetected).debounce(0.05);

    private Timer intakingTimer = new Timer();

    private Angle goal = IntakeConstants.STOW_POS;

    IntakeVisualizer visualizer;

    public Intake(IntakeIO io, Supplier<Pose2d> poseSupplier) {
        this.io = io;
        this.visualizer = new IntakeVisualizer(coralDetectedTrigger, poseSupplier);

        this.deployedTrigger.onTrue(this.stopIntake());

        SmartDashboard.putData("Intake Deploy", deployCommand(true));
        SmartDashboard.putData("Intake Stow", stowCommand(true));
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Intake", inputs);
        visualizer.update(inputs.position);
    }

    public void setIntakeSpeed(Voltage speed) {
        io.setIntakeSpeed(speed);
    }

    public void setAlignSpeed(Voltage speed) {
        io.setAlignSpeed(speed);
    }

    public void setGoal(Angle angle) {
        goal = angle;
        io.setGoal(angle);
    }

    public boolean alignerHasPiece() {
        return alignerHasPiece.getAsBoolean();
    }

    public Angle getPosition() {
        return inputs.position;
    }

    public boolean isDeployed() {
        return inputs.position.isNear(IntakeConstants.DEPLOY_POS, IntakeConstants.DEPLOY_TOLERANCE);
    }

    public boolean isStowed() {
        return inputs.position.isNear(IntakeConstants.STOW_POS, IntakeConstants.STOW_TOLERANCE);
    }

    private boolean rawCoralDetected() {
        return inputs.alignLidar || inputs.coralDetected;
    }

    public Command toggleCommand(boolean instant) {
        if (goal.lte(IntakeConstants.DEPLOY_TOLERANCE)) {
            return stowCommand(instant);
        } else {
            return deployCommand(instant);
        }
    }

    public Command deployCommand(boolean instant) {
        if (instant) {
            return this.runOnce(() -> setGoal(IntakeConstants.DEPLOY_POS));
        } else {
            return this.runOnce(() -> setGoal(IntakeConstants.DEPLOY_POS)).andThen(Commands.waitUntil(deployedTrigger));
        }
    }

    public Command stowCommand(boolean instant) {
        if (instant) {
            return this.runOnce(() -> setGoal(IntakeConstants.STOW_POS));
        } else {
            return this.runOnce(() -> setGoal(IntakeConstants.STOW_POS)).andThen(Commands.waitUntil(stowedTrigger));
        }
    }

    public Command startIntakeCommand() {
        return Commands.runOnce(() -> {
                    intakingTimer.restart();
                    setIntakeSpeed(IntakeConstants.INTAKE_SPEED);
                    setAlignSpeed(IntakeConstants.ALIGN_SPEED);
                })
                .withName("Start Intake Command");
    }

    public Command startSlowIntakeCommand() {
        return Commands.runOnce(() -> {
                    intakingTimer.restart();
                    setIntakeSpeed(IntakeConstants.SLOW_INTAKE_SPEED);
                    setAlignSpeed(IntakeConstants.ALIGN_SPEED);
                })
                .withName("Start Intake Command");
    }

    public Command startEjectCommand() {
        return Commands.runOnce(() -> {
                    setIntakeSpeed(IntakeConstants.EJECT_SPEED);
                    setAlignSpeed(IntakeConstants.EJECT_SPEED);
                })
                .withName("Start Eject Command");
    }

    public Command intakeCommand() {
        return Commands.startEnd(
                        () -> {
                            intakingTimer.restart();
                            setIntakeSpeed(IntakeConstants.INTAKE_SPEED);
                            setAlignSpeed(IntakeConstants.ALIGN_SPEED);
                        },
                        () -> {
                            setIntakeSpeed(Volts.zero());
                            setAlignSpeed(Volts.zero());
                        })
                .withName("Intake Command");
    }

    public Command ejectCommand() {
        return Commands.startEnd(
                        () -> {
                            setIntakeSpeed(IntakeConstants.EJECT_SPEED);
                            setAlignSpeed(IntakeConstants.EJECT_SPEED);
                        },
                        () -> {
                            setIntakeSpeed(Volts.zero());
                            setAlignSpeed(Volts.zero());
                        })
                .withName("Eject Command");
    }

    private Command stopDeploy() {
        return Commands.runOnce(() -> io.stopDeploy());
    }

    public Command stopIntake() {
        return Commands.runOnce(() -> {
            setIntakeSpeed(Volts.zero());
            setAlignSpeed(Volts.zero());
        });
    }
}
