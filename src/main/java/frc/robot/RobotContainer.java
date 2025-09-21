// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.RobotController;
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
import frc.robot.commands.ApproachBargeCommands;
import frc.robot.commands.ApproachReefCommand;
import frc.robot.commands.AutoCoralCommand;
import frc.robot.commands.FullAutoCommand;
import frc.robot.commands.LollipopCommands;
import frc.robot.commands.ProcessCommand;
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
import frc.robot.subsystems.coraldetection.CoralDetection;
import frc.robot.subsystems.coraldetection.CoralDetectionIO;
import frc.robot.subsystems.coraldetection.CoralDetectionIOPhotonVision;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorIO;
import frc.robot.subsystems.elevator.ElevatorIOSim;
import frc.robot.subsystems.elevator.ElevatorIOTalonFX;
import frc.robot.subsystems.endeffector.EndEffector;
import frc.robot.subsystems.endeffector.EndEffectorIO;
import frc.robot.subsystems.endeffector.EndEffectorIOSim;
import frc.robot.subsystems.endeffector.EndEffectorIOTalonFX;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeIO;
import frc.robot.subsystems.intake.IntakeIOSim;
import frc.robot.subsystems.intake.IntakeIOTalonFX;
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
import frc.robot.util.Elastic;
import frc.robot.util.Elastic.Notification;
import frc.robot.util.Elastic.Notification.NotificationLevel;
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
    private final Intake intake;
    private final CoralDetection coralDetection;

    // Commands
    private final TeleopSwerve teleopSwerveCommand;

    private final Command rumbleCommand;
    private final Command reefCommand;
    private final Command coralSearchCommand;
    private final Command algaeCommand;
    private final Command bargeCommand;
    private final Command sweepCommand;
    private final Command processCommand;
    private final Map<Character, Supplier<Command>> elevatorCommands;

    @SuppressWarnings("unused")
    private final Command elevatorCommand;

    private final Command lollipopCommand;

    private final AutoCoralCommand autoCoralCommand;

    // Triggers
    private final Trigger algaeButtonLayerTrigger = driveController.leftTrigger();
    private final Trigger speedUpTrigger = driveController.rightTrigger();
    // private final Trigger slowDownTrigger = driveController.leftTrigger();

    private final Trigger elevatorUpTrigger = driveController.povUp();
    private final Trigger elevatorDownTrigger = driveController.povDown();
    private final Trigger elevatorIntakeTrigger = driveController.povLeft();
    // private final Trigger elevatorTrigger = driveController.povRight();

    private final Trigger intakeTrigger = driveController.rightBumper().and(algaeButtonLayerTrigger.negate());
    private final Trigger reverseIntakeTrigger = driveController.leftBumper().and(algaeButtonLayerTrigger.negate());

    private final Trigger toggleIntakeDeployTrigger = driveController.povRight();
    private final Trigger autoCoralTrigger = driveController.y().and(algaeButtonLayerTrigger.negate());

    private final Trigger reefTrigger = driveController.a();
    private final Trigger coralSearchTrigger = driveController.b().and(algaeButtonLayerTrigger.negate());
    private final Trigger sweepTrigger = driveController.x().and(algaeButtonLayerTrigger.negate());
    private final Trigger autoClimbTrigger = driveController.start();
    private final Trigger unclimbTrigger = driveController.back();
    private final Trigger climbGrabPosTrigger = buttonBoardOther.button(2);
    private final Trigger algaeAndCoralToggle = algaeButtonLayerTrigger;

    private final Trigger algaeTrigger = driveController.y().and(algaeButtonLayerTrigger);
    private final Trigger bargeTrigger = driveController.b().and(algaeButtonLayerTrigger);
    private final Trigger lollipopTrigger = driveController.x().and(algaeButtonLayerTrigger);
    private final Trigger algaeManualIntakeTrigger =
            driveController.rightBumper().and(algaeButtonLayerTrigger);
    private final Trigger algaeManualEjectTrigger = driveController.leftBumper().and(algaeButtonLayerTrigger);

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

    private final Map<Integer, String> BUTTON_TO_SEARCH_POS = Map.ofEntries(
            Map.entry(3, "S1"),
            Map.entry(4, "S1"),
            Map.entry(5, "S1"),
            Map.entry(6, "S0"),
            Map.entry(7, "S0"),
            Map.entry(8, "S0"));

    private final Trigger[] reefTriggers = new Trigger[12];
    private final Trigger[] levelTriggers = new Trigger[] {
        buttonBoardOther.button(9),
        buttonBoardOther.button(10),
        buttonBoardOther.button(11),
        buttonBoardOther.button(12)
    };

    private final Trigger[] coralSearchTriggers = new Trigger[6];

    private final Alert lowBatteryAlert = new Alert("Low Battery Voltage!", AlertType.kError);
    private final Notification lowBatteryNotification = new Notification(
            NotificationLevel.ERROR,
            "Low Battery Voltage!",
            "The robot's battery voltage is under " + Constants.LOW_BATTERY_VOLTAGE.in(Volts)
                    + "V. Please change the battery.",
            40000);

    private final Alert driveControllerDisconnected = new Alert("Drive Controller Disconnected!", AlertType.kError);
    private final Alert buttonBoard1Disconnected = new Alert("Button Board 1 Disconnected!", AlertType.kError);
    private final Alert buttonBoard2Disconnected = new Alert("Button Board 2 Disconnected!", AlertType.kError);
    private final Alert noAutoSelected = new Alert("No Auto Selected!", AlertType.kWarning);

    private final Trigger endgameTrigger =
            new Trigger(() -> (DriverStation.isTeleop() && DriverStation.getMatchTime() < 20));

    public RobotContainer() {
        AlignToReefCommands.testReefPoses();
        AlignToReefCommands.testBranchPoses();
        ApproachBargeCommands.testBargePoses();

        NTConstants.AUTO_DESCRIPTOR_ENTRY.set("");
        NTConstants.REEF_TELEOP_AUTO_ENTRY.set("A4");
        NTConstants.SEARCH_POS_TELEOP_AUTO_ENTRY.set("S0");

        switch (Constants.CURRENT_MODE) {
            case REAL:
                swerve = new Swerve(
                        new GyroIOPigeon2(),
                        new ModuleIOTalonFX(SwerveConstants.FrontLeft.MODULE_CONSTANTS),
                        new ModuleIOTalonFX(SwerveConstants.FrontRight.MODULE_CONSTANTS),
                        new ModuleIOTalonFX(SwerveConstants.BackLeft.MODULE_CONSTANTS),
                        new ModuleIOTalonFX(SwerveConstants.BackRight.MODULE_CONSTANTS));

                endEffector = new EndEffector(new EndEffectorIOTalonFX());
                algaeManipulator = new AlgaeManipulator(new AlgaeManipulatorIOTalonFX());
                elevator = new Elevator(
                        new ElevatorIOTalonFX(),
                        endEffector.coralDetectedTrigger,
                        algaeManipulator.deployedTrigger,
                        algaeManipulator.algaeDetectedTrigger,
                        swerve::getPose);
                climber = new Climber(new ClimberIOTalonFX());
                intake = new Intake(new IntakeIOTalonFX(), swerve::getPose);
                coralDetection = new CoralDetection(
                        new CoralDetectionIOPhotonVision(VisionConstants.CORAL_CAM_NAME, VisionConstants.CORAL_CAM_POS),
                        swerve::getPose);

                vision = new Vision(
                        swerve::addVisionMeasurement,
                        new VisionIOPhotonVision(
                                VisionConstants.FRONT_LEFT_CAM_NAME, VisionConstants.FRONT_LEFT_CAM_POS),
                        new VisionIOPhotonVision(
                                VisionConstants.FRONT_RIGHT_CAM_NAME, VisionConstants.FRONT_RIGHT_CAM_POS));

                if (RobotController.getBatteryVoltage() < Constants.LOW_BATTERY_VOLTAGE.in(Volts)) {
                    lowBatteryAlert.set(true);
                    Elastic.sendNotification(lowBatteryNotification);
                }
                break;

            case SIM:
                swerve = new Swerve(
                        new GyroIO() {},
                        new ModuleIOSim(SwerveConstants.FrontLeft.MODULE_CONSTANTS),
                        new ModuleIOSim(SwerveConstants.FrontRight.MODULE_CONSTANTS),
                        new ModuleIOSim(SwerveConstants.BackLeft.MODULE_CONSTANTS),
                        new ModuleIOSim(SwerveConstants.BackRight.MODULE_CONSTANTS));

                endEffector = new EndEffector(new EndEffectorIOSim());
                algaeManipulator = new AlgaeManipulator(new AlgaeManipulatorIOSim());
                elevator = new Elevator(
                        new ElevatorIOSim(),
                        endEffector.coralDetectedTrigger,
                        algaeManipulator.deployedTrigger,
                        algaeManipulator.algaeDetectedTrigger,
                        swerve::getPose);
                climber = new Climber(new ClimberIOSim());
                intake = new Intake(new IntakeIOSim(), swerve::getPose);
                coralDetection = new CoralDetection(new CoralDetectionIO() {}, swerve::getPose);

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
                algaeManipulator = new AlgaeManipulator(new AlgaeManipulatorIO() {});
                elevator = new Elevator(
                        new ElevatorIO() {},
                        endEffector.coralDetectedTrigger,
                        algaeManipulator.deployedTrigger,
                        algaeManipulator.algaeDetectedTrigger,
                        swerve::getPose);
                climber = new Climber(new ClimberIO() {});
                intake = new Intake(new IntakeIO() {}, swerve::getPose);
                coralDetection = new CoralDetection(new CoralDetectionIO() {}, swerve::getPose);

                vision = new Vision(swerve::addVisionMeasurement, new VisionIO() {}, new VisionIO() {});
                break;
        }

        teleopSwerveCommand = new TeleopSwerve(swerve, driveController);

        rumbleCommand = Commands.startEnd(
                        () -> driveController.setRumble(RumbleType.kBothRumble, ControllerConstants.CONTROLLER_RUMBLE),
                        () -> driveController.setRumble(RumbleType.kBothRumble, 0))
                .withName("Rumble");

        reefCommand = Commands.defer(
                        () -> new AutoCoralCommand(swerve, intake, endEffector, elevator, coralDetection)
                                .andThen(new FullAutoCommand(
                                        NTConstants.REEF_TELEOP_AUTO_ENTRY.get(),
                                        swerve,
                                        elevator,
                                        endEffector,
                                        algaeManipulator,
                                        intake,
                                        coralDetection)),
                        Set.of(swerve, elevator))
                .andThen(rumbleCommand.asProxy().withTimeout(ControllerConstants.SCORE_RUMBLE_TIME))
                .withName("Reef Auto");

        coralSearchCommand = Commands.defer(
                        () -> new FullAutoCommand(
                                NTConstants.SEARCH_POS_TELEOP_AUTO_ENTRY.get(),
                                swerve,
                                elevator,
                                endEffector,
                                algaeManipulator,
                                intake,
                                coralDetection),
                        Set.of(swerve, elevator))
                .withName("Coral Search Auto");

        algaeCommand = Commands.defer(
                        () -> new RemoveAlgaeCommand(
                                NTConstants.REEF_TELEOP_AUTO_ENTRY.get(), swerve, elevator, algaeManipulator),
                        Set.of(swerve, elevator))
                .withName("Algae Auto");

        bargeCommand = Commands.defer(
                        () -> new ScoreInBargeCommand(
                                NTConstants.REEF_TELEOP_AUTO_ENTRY.get().charAt(0), swerve, elevator, algaeManipulator),
                        Set.of(swerve, elevator))
                .withName("Barge Auto");

        lollipopCommand = Commands.defer(
                        () -> LollipopCommands.lollipopCommand(swerve, algaeManipulator, elevator),
                        Set.of(swerve, elevator))
                .withName("Lollipop Auto");

        processCommand = Commands.defer(
                        () -> new ProcessCommand(swerve, elevator, algaeManipulator), Set.of(swerve, elevator))
                .withName("Process");

        SmartDashboard.putData("Coral Search Auto", coralSearchCommand);
        SmartDashboard.putData("Reef Auto", reefCommand);
        SmartDashboard.putData("Algae Auto", algaeCommand);
        SmartDashboard.putData("Barge Auto", bargeCommand);
        SmartDashboard.putData("Nearest Lollipop", lollipopCommand);
        SmartDashboard.putData("Process", processCommand);

        ApproachReefCommand approach = new ApproachReefCommand(0, 1, swerve);
        SmartDashboard.putData("Track L3", elevator.trackL3Command(approach::getDistanceToTarget)); // for debug

        sweepCommand =
                Commands.defer(() -> new SweepCommand(swerve), Set.of(swerve)).withName("Sweep");

        elevatorCommands = Map.ofEntries(
                Map.entry('1', () -> elevator.goToL1Command(false)),
                Map.entry('2', () -> elevator.goToL2Command(false)),
                Map.entry('3', () -> elevator.goToL3Command(false)),
                Map.entry('4', () -> elevator.goToL4Command(false)));

        elevatorCommand = Commands.defer(
                        () -> elevatorCommands
                                .get(NTConstants.REEF_TELEOP_AUTO_ENTRY.get().charAt(1))
                                .get(),
                        Set.of(elevator))
                .withName("Elevator Command");

        swerve.setDefaultCommand(teleopSwerveCommand);

        endgameTrigger.onTrue(Commands.runOnce(() -> Elastic.selectTab("Endgame")));

        autoCoralCommand = new AutoCoralCommand(swerve, intake, endEffector, elevator, coralDetection);

        configureBindings();
    }

    private void configureBindings() {
        speedUpTrigger.whileTrue(teleopSwerveCommand.speedUpCommand());
        // slowDownTrigger.whileTrue(teleopSwerveCommand.slowDownCommand());

        elevatorUpTrigger.whileTrue(elevator.elevatorUpCommand());
        elevatorDownTrigger.whileTrue(elevator.elevatorDownCommand());
        elevatorIntakeTrigger.whileTrue(elevator.goToIntakePosCommand(false));
        // elevatorTrigger.whileTrue(elevatorCommand);

        intakeTrigger.whileTrue(endEffector
                .runCommand(EndEffectorConstants.INTAKE_SPEED)
                .alongWith(intake.intakeCommand().onlyIf(intake::isDeployed)));
        reverseIntakeTrigger.whileTrue(endEffector
                .runCommand(EndEffectorConstants.INTAKE_SPEED.unaryMinus())
                .alongWith(intake.ejectCommand()));

        toggleIntakeDeployTrigger.onTrue(Commands.defer(() -> intake.toggleCommand(false), Set.of(intake)));
        autoCoralTrigger.whileTrue(autoCoralCommand);

        reefTrigger.whileTrue(reefCommand);
        coralSearchTrigger.whileTrue(coralSearchCommand);
        algaeTrigger.whileTrue(algaeCommand);
        bargeTrigger.whileTrue(bargeCommand);
        sweepTrigger.whileTrue(sweepCommand);
        lollipopTrigger.whileTrue(lollipopCommand);

        algaeManualIntakeTrigger.whileTrue(algaeManipulator.forwardCommand());
        algaeManualEjectTrigger.whileTrue(algaeManipulator.reverseCommand());

        algaeAndCoralToggle.whileTrue(setAlgaeCommand());

        autoClimbTrigger.whileTrue(climber.autoClimbCommand());
        unclimbTrigger.whileTrue(climber.reverseCommand());
        climbGrabPosTrigger.whileTrue(climber.goToGrabPosCommand());

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

        for (int i = 0; i < coralSearchTriggers.length; i++) {
            final int button = i + 3;
            coralSearchTriggers[i] = buttonBoardOther.button(button);
            coralSearchTriggers[i].onTrue(
                    new InstantCommand(() -> setTeleopAutoDescriptorSearch(BUTTON_TO_SEARCH_POS.get(button)))
                            .ignoringDisable(true));
        }

        // intake.coralDetectedTrigger.whileTrue(rumbleCommand);
    }

    public void updateAlerts() {
        driveControllerDisconnected.set(
                !DriverStation.isJoystickConnected(driveController.getHID().getPort()));
        buttonBoard1Disconnected.set(
                !DriverStation.isJoystickConnected(buttonBoardReef.getHID().getPort()));
        buttonBoard2Disconnected.set(
                !DriverStation.isJoystickConnected(buttonBoardOther.getHID().getPort()));

        noAutoSelected.set(NTConstants.AUTO_DESCRIPTOR_ENTRY.get().equals(""));
    }

    private void setTeleopAutoDescriptorLetter(char letter) {
        String descriptor = NTConstants.REEF_TELEOP_AUTO_ENTRY.get();
        NTConstants.REEF_TELEOP_AUTO_ENTRY.set(letter + descriptor.substring(1));
    }

    private void setTeleopAutoDescriptorLevel(int level) {
        String descriptor = NTConstants.REEF_TELEOP_AUTO_ENTRY.get();
        NTConstants.REEF_TELEOP_AUTO_ENTRY.set(descriptor.substring(0, 1) + level);
    }

    private void setTeleopAutoDescriptorSearch(String pos) {
        NTConstants.SEARCH_POS_TELEOP_AUTO_ENTRY.set(pos);
    }

    private void setAlgaeAutoDescriptor(boolean doAlgae) {
        String descriptor = NTConstants.REEF_TELEOP_AUTO_ENTRY.get();
        NTConstants.REEF_TELEOP_AUTO_ENTRY.set(descriptor.substring(0, 2) + (doAlgae ? "A" : ""));
    }

    private Command setAlgaeCommand() {
        return Commands.startEnd(() -> setAlgaeAutoDescriptor(true), () -> setAlgaeAutoDescriptor(false))
                .withName("Toggle Algae");
    }

    public Command getAutonomousCommand() {
        return new FullAutoCommand(
                NTConstants.AUTO_DESCRIPTOR_ENTRY.get(),
                swerve,
                elevator,
                endEffector,
                algaeManipulator,
                intake,
                coralDetection);
    }
}
