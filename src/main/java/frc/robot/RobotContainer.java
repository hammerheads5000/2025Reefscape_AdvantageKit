// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.EndEffectorConstants;
import frc.robot.Constants.NTConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.commands.AlignToReefCommands;
import frc.robot.commands.ApproachCoralStationCommands;
import frc.robot.commands.ApproachReefCommand;
import frc.robot.commands.FullAutoCommand;
import frc.robot.commands.RemoveAlgaeCommand;
import frc.robot.commands.ScoreInBargeCommand;
import frc.robot.commands.SweepCommand;
import frc.robot.commands.TeleopSwerve;
import frc.robot.subsystems.algaemanipulator.AlgaeManipulator;
import frc.robot.subsystems.algaemanipulator.AlgaeManipulatorIO;
import frc.robot.subsystems.algaemanipulator.AlgaeManipulatorIOSim;
import frc.robot.subsystems.algaemanipulator.AlgaeManipulatorIOTalonFX;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.climber.ClimberIO;
import frc.robot.subsystems.climber.ClimberIOSim;
import frc.robot.subsystems.climber.ClimberIOTalonFX;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorIO;
import frc.robot.subsystems.elevator.ElevatorIOSim;
import frc.robot.subsystems.elevator.ElevatorIOTalonFX;
import frc.robot.subsystems.endeffector.EndEffector;
import frc.robot.subsystems.endeffector.EndEffectorIO;
import frc.robot.subsystems.endeffector.EndEffectorIOSim;
import frc.robot.subsystems.endeffector.EndEffectorIOTalonFX;
import frc.robot.subsystems.swerve.GyroIO;
import frc.robot.subsystems.swerve.GyroIOPigeon2;
import frc.robot.subsystems.swerve.ModuleIO;
import frc.robot.subsystems.swerve.ModuleIOSim;
import frc.robot.subsystems.swerve.ModuleIOTalonFX;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOPhotonVision;
import frc.robot.subsystems.vision.VisionIOPhotonVisionSim;
import java.util.Map;
import java.util.Set;
import java.util.function.Supplier;

public class RobotContainer {
    // Controllers
    private final CommandXboxController driveController = new CommandXboxController(0);
    private final CommandGenericHID buttonBoardReef = new CommandGenericHID(1);
    private final CommandGenericHID buttonBoardOther = new CommandGenericHID(2);

    // Subsystems
    @SuppressWarnings("unused")
    private final Vision vision;

    private final Swerve swerve;
    public final Elevator elevator;
    private final EndEffector endEffector;
    private final Climber climber;
    private final AlgaeManipulator algaeManipulator;

    // Commands
    private final TeleopSwerve teleopSwerveCommand;

    private final Command rumbleCommand;
    private final Command reefCommand;
    private final Command stationCommand;
    private final Command algaeCommand;
    private final Command bargeCommand;
    private final Command sweepCommand;
    private final Map<Character, Supplier<Command>> elevatorCommands;
    private final Command elevatorCommand;

    // Triggers
    private final Trigger speedUpTrigger = driveController.rightTrigger();
    private final Trigger slowDownTrigger = driveController.leftTrigger();

    private final Trigger elevatorUpTrigger = driveController.povUp();
    private final Trigger elevatorDownTrigger = driveController.povDown();
    private final Trigger elevatorIntakeTrigger = driveController.povLeft();
    // private final Trigger elevatorTrigger = driveController.povRight();

    private final Trigger intakeTrigger = driveController.rightBumper();
    private final Trigger reverseIntakeTrigger = driveController.leftBumper();

    private final Trigger reefTrigger = driveController.a();
    private final Trigger stationTrigger = driveController.b();
    private final Trigger algaeTrigger = driveController.y();
    private final Trigger bargeTrigger = driveController.x();
    private final Trigger sweepTrigger = driveController.povRight();
    private final Trigger climbTrigger = driveController.start();
    private final Trigger unclimbTrigger = driveController.back();
    private final Trigger algaeAndCoralToggle = buttonBoardOther.button(2);

    private final Map<Integer, Character> BUTTON_TO_REEF = Map.ofEntries(
            Map.entry(5, 'A'),
            Map.entry(7, 'B'),
            Map.entry(9, 'C'),
            Map.entry(11, 'D'),
            Map.entry(12, 'E'),
            Map.entry(10, 'F'),
            Map.entry(8, 'G'),
            Map.entry(6, 'H'),
            Map.entry(4, 'I'),
            Map.entry(2, 'J'),
            Map.entry(1, 'K'),
            Map.entry(3, 'L'));

    private final Map<Integer, String> BUTTON_TO_STATION = Map.ofEntries(
            Map.entry(3, "S1L"),
            Map.entry(4, "S1C"),
            Map.entry(5, "S1R"),
            Map.entry(6, "S0L"),
            Map.entry(7, "S0C"),
            Map.entry(8, "S0R"));

    private final Trigger[] reefTriggers = new Trigger[12];
    private final Trigger[] levelTriggers = new Trigger[] {
        buttonBoardOther.button(9),
        buttonBoardOther.button(10),
        buttonBoardOther.button(11),
        buttonBoardOther.button(12)
    };

    private final Trigger[] stationTriggers = new Trigger[6];

    public RobotContainer() {
        AlignToReefCommands.testReefPoses();
        ApproachCoralStationCommands.testStationPoses();

        NTConstants.AUTO_DESCRIPTOR_ENTRY.set("");
        NTConstants.REEF_TELEOP_AUTO_ENTRY.set("A4");
        NTConstants.STATION_TELEOP_AUTO_ENTRY.set("S0");

        switch (Constants.CURRENT_MODE) {
            case REAL:
                swerve = new Swerve(
                        new GyroIOPigeon2(),
                        new ModuleIOTalonFX(SwerveConstants.FrontLeft.MODULE_CONSTANTS),
                        new ModuleIOTalonFX(SwerveConstants.FrontRight.MODULE_CONSTANTS),
                        new ModuleIOTalonFX(SwerveConstants.BackLeft.MODULE_CONSTANTS),
                        new ModuleIOTalonFX(SwerveConstants.BackRight.MODULE_CONSTANTS));

                endEffector = new EndEffector(new EndEffectorIOTalonFX());
                elevator = new Elevator(new ElevatorIOTalonFX(), endEffector.hasCoralTrigger, swerve::getPose);
                climber = new Climber(new ClimberIOTalonFX());
                algaeManipulator = new AlgaeManipulator(new AlgaeManipulatorIOTalonFX());

                vision = new Vision(
                        swerve::addVisionMeasurement,
                        new VisionIOPhotonVision(
                                VisionConstants.FRONT_LEFT_CAM_NAME, VisionConstants.FRONT_LEFT_CAM_POS),
                        new VisionIOPhotonVision(
                                VisionConstants.FRONT_RIGHT_CAM_NAME, VisionConstants.FRONT_RIGHT_CAM_POS));
                break;

            case SIM:
                swerve = new Swerve(
                        new GyroIO() {},
                        new ModuleIOSim(SwerveConstants.FrontLeft.MODULE_CONSTANTS),
                        new ModuleIOSim(SwerveConstants.FrontRight.MODULE_CONSTANTS),
                        new ModuleIOSim(SwerveConstants.BackLeft.MODULE_CONSTANTS),
                        new ModuleIOSim(SwerveConstants.BackRight.MODULE_CONSTANTS));

                endEffector = new EndEffector(new EndEffectorIOSim());
                elevator = new Elevator(new ElevatorIOSim(), endEffector.hasCoralTrigger, swerve::getPose);
                climber = new Climber(new ClimberIOSim());
                algaeManipulator = new AlgaeManipulator(new AlgaeManipulatorIOSim());

                vision = new Vision(
                        swerve::addVisionMeasurement,
                        new VisionIOPhotonVisionSim(
                                VisionConstants.FRONT_LEFT_CAM_NAME,
                                VisionConstants.FRONT_LEFT_CAM_POS,
                                swerve::getPose),
                        new VisionIOPhotonVisionSim(
                                VisionConstants.FRONT_RIGHT_CAM_NAME,
                                VisionConstants.FRONT_RIGHT_CAM_POS,
                                swerve::getPose));
                break;

            default:
                swerve = new Swerve(
                        new GyroIO() {}, new ModuleIO() {}, new ModuleIO() {}, new ModuleIO() {}, new ModuleIO() {});

                endEffector = new EndEffector(new EndEffectorIO() {});
                elevator = new Elevator(new ElevatorIO() {}, endEffector.hasCoralTrigger, swerve::getPose);
                climber = new Climber(new ClimberIO() {});
                algaeManipulator = new AlgaeManipulator(new AlgaeManipulatorIO() {});

                vision = new Vision(swerve::addVisionMeasurement, new VisionIO() {}, new VisionIO() {});
                break;
        }

        teleopSwerveCommand = new TeleopSwerve(swerve, driveController);

        rumbleCommand = Commands.startEnd(
                () -> driveController.setRumble(RumbleType.kBothRumble, ControllerConstants.CONTROLLER_RUMBLE),
                () -> driveController.setRumble(RumbleType.kBothRumble, 0));

        reefCommand = Commands.defer(
                        () -> new FullAutoCommand(
                                NTConstants.REEF_TELEOP_AUTO_ENTRY.get(),
                                swerve,
                                elevator,
                                endEffector,
                                algaeManipulator),
                        Set.of(swerve, elevator))
                .andThen(rumbleCommand.asProxy().withTimeout(ControllerConstants.SCORE_RUMBLE_TIME));

        stationCommand = Commands.defer(
                () -> new FullAutoCommand(
                        NTConstants.STATION_TELEOP_AUTO_ENTRY.get(), swerve, elevator, endEffector, algaeManipulator),
                Set.of(swerve, elevator));

        algaeCommand = Commands.defer(
                () -> new RemoveAlgaeCommand(
                        NTConstants.REEF_TELEOP_AUTO_ENTRY.get(), swerve, elevator, algaeManipulator),
                Set.of(swerve, elevator, algaeManipulator));

        bargeCommand = Commands.defer(
                () -> new ScoreInBargeCommand(
                        NTConstants.REEF_TELEOP_AUTO_ENTRY.get().charAt(1), swerve, elevator, algaeManipulator),
                Set.of(swerve, elevator, algaeManipulator));

        SmartDashboard.putData("Station Auto", stationCommand);
        SmartDashboard.putData("Reef Auto", reefCommand);
        SmartDashboard.putData("Algae Auto", algaeCommand);
        SmartDashboard.putData("Barge Auto", bargeCommand);

        ApproachReefCommand approach = new ApproachReefCommand(0, 1, swerve);
        SmartDashboard.putData("Track L3", elevator.trackL3Command(approach::getDistanceToTarget)); // for debug

        sweepCommand = Commands.defer(() -> new SweepCommand(swerve), Set.of(swerve));

        elevatorCommands = Map.ofEntries(
                Map.entry('1', () -> elevator.goToL1Command(false)),
                Map.entry('2', () -> elevator.goToL2Command(false)),
                Map.entry('3', () -> elevator.goToL3Command(false)),
                Map.entry('4', () -> elevator.goToL4Command(false)));

        elevatorCommand = Commands.defer(
                () -> elevatorCommands
                        .get(NTConstants.REEF_TELEOP_AUTO_ENTRY.get().charAt(1))
                        .get(),
                Set.of(elevator));

        swerve.setDefaultCommand(teleopSwerveCommand);

        configureBindings();
    }

    private void configureBindings() {
        speedUpTrigger.whileTrue(teleopSwerveCommand.speedUpCommand());
        slowDownTrigger.whileTrue(teleopSwerveCommand.slowDownCommand());

        elevatorUpTrigger.whileTrue(elevator.elevatorUpCommand());
        elevatorDownTrigger.whileTrue(elevator.elevatorDownCommand());
        elevatorIntakeTrigger.whileTrue(elevator.goToIntakePosCommand(false));
        // elevatorTrigger.whileTrue(elevatorCommand);

        intakeTrigger.whileTrue(endEffector.runCommand(EndEffectorConstants.INTAKE_SPEED));
        reverseIntakeTrigger.whileTrue(endEffector.runCommand(EndEffectorConstants.INTAKE_SPEED.unaryMinus()));

        reefTrigger.whileTrue(reefCommand);
        stationTrigger.whileTrue(stationCommand);
        algaeTrigger.whileTrue(algaeCommand);
        bargeTrigger.whileTrue(bargeCommand);
        sweepTrigger.whileTrue(sweepCommand);

        algaeAndCoralToggle.whileTrue(setAlgaeCommand());

        climbTrigger.whileTrue(climber.climbCommand());
        unclimbTrigger.whileTrue(climber.reverseCommand());

        for (int i = 0; i < reefTriggers.length; i++) {
            reefTriggers[i] = buttonBoardReef.button(i + 1);

            final char branch = BUTTON_TO_REEF.get(i + 1);
            reefTriggers[i].onTrue(
                    new InstantCommand(() -> setTeleopAutoDescriptorLetter(branch)).ignoringDisable(true));
        }

        for (int i = 0; i < levelTriggers.length; i++) {
            final int level = i + 1;
            levelTriggers[i].onTrue(
                    new InstantCommand(() -> setTeleopAutoDescriptorLevel(level)).ignoringDisable(true));
        }

        for (int i = 0; i < stationTriggers.length; i++) {
            final int button = i + 3;
            stationTriggers[i] = buttonBoardOther.button(button);
            stationTriggers[i].onTrue(
                    new InstantCommand(() -> setTeleopAutoDescriptorStation(BUTTON_TO_STATION.get(button)))
                            .ignoringDisable(true));
        }

        endEffector.coralDetectedTrigger.whileTrue(rumbleCommand);
    }

    private void setTeleopAutoDescriptorLetter(char letter) {
        String descriptor = NTConstants.REEF_TELEOP_AUTO_ENTRY.get();
        NTConstants.REEF_TELEOP_AUTO_ENTRY.set(letter + descriptor.substring(1));
    }

    private void setTeleopAutoDescriptorLevel(int level) {
        String descriptor = NTConstants.REEF_TELEOP_AUTO_ENTRY.get();
        NTConstants.REEF_TELEOP_AUTO_ENTRY.set(descriptor.substring(0, 1) + level);
    }

    private void setTeleopAutoDescriptorStation(String station) {
        NTConstants.STATION_TELEOP_AUTO_ENTRY.set(station);
    }

    private void setAlgaeAutoDescriptor(boolean doAlgae) {
        String descriptor = NTConstants.AUTO_DESCRIPTOR_ENTRY.get();
        NTConstants.REEF_TELEOP_AUTO_ENTRY.set(descriptor.substring(0, 2) + (doAlgae ? "A" : ""));
    }

    private Command setAlgaeCommand() {
        return Commands.startEnd(() -> setAlgaeAutoDescriptor(true), () -> setAlgaeAutoDescriptor(false));
    }

    public Command getAutonomousCommand() {
        return new FullAutoCommand(
                NTConstants.AUTO_DESCRIPTOR_ENTRY.get(), swerve, elevator, endEffector, algaeManipulator);
    }
}
