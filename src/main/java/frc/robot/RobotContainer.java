// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

// #region Imports
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.filter.Debouncer.DebounceType;
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
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.NTConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.commands.AlignToReefCommands;
import frc.robot.commands.ApproachBargeCommands;
import frc.robot.commands.ApproachReefCommand;
import frc.robot.commands.AutoCoralCommand;
import frc.robot.commands.ClimbSequence;
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
import frc.robot.subsystems.coraldetection.CoralDetectionIOSim;
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
// #endregion

public class RobotContainer {
    // Controllers
    private final CommandXboxController driveController = new CommandXboxController(0);
    private final CommandGenericHID buttonBoardReef = new CommandGenericHID(1);
    private final CommandGenericHID buttonBoardOther = new CommandGenericHID(2);

    // #region Subsystems
    @SuppressWarnings("unused")
    private final Vision vision;

    private final Swerve swerve;
    public final Elevator elevator;
    private final EndEffector endEffector;
    private final Climber climber;
    private final AlgaeManipulator algaeManipulator;
    public final Intake intake;
    private final CoralDetection coralDetection;
    // #endregion

    // #region Commands
    private final TeleopSwerve teleopSwerveCommand;

    private final Command rumbleCommand;
    private final Command intakeAndReefCommand;
    private final Command reefCommand;
    private final Command algaeCommand;
    private final Command bargeCommand;
    private final Command sweepCommand;
    private final Command processCommand;
    private final Map<Character, Supplier<Command>> elevatorCommands;

    @SuppressWarnings("unused")
    private final Command elevatorCommand;

    private final Command lollipopCommand;

    private final AutoCoralCommand autoCoralCommand;

    private final ClimbSequence climbSequence;
    // #endregion

    // #region Triggers
    private final Trigger algaeButtonLayerTrigger = driveController.leftTrigger();
    // private final Trigger speedUpTrigger = driveController.rightTrigger();
    // private final Trigger slowDownTrigger = driveController.leftTrigger();

    private final Trigger elevatorUpTrigger = driveController.povUp();
    private final Trigger elevatorDownTrigger = driveController.povDown();
    private final Trigger elevatorIntakeTrigger = driveController.povLeft();
    // private final Trigger elevatorTrigger = driveController.povRight();

    private final Trigger intakeTrigger = driveController.rightTrigger().and(algaeButtonLayerTrigger.negate());
    private final Trigger shootTrigger = driveController.rightBumper().and(algaeButtonLayerTrigger.negate());
    private final Trigger reverseIntakeTrigger = driveController.leftBumper().and(algaeButtonLayerTrigger.negate());

    private final Trigger toggleIntakeDeployTrigger = driveController.povRight();
    private final Trigger elevatorZeroTrigger = buttonBoardOther.button(1);
    private final Trigger autoCoralTrigger = driveController.y().and(algaeButtonLayerTrigger.negate());

    private final Trigger intakeAndReefTrigger = driveController.a();
    private final Trigger reefTrigger = driveController.b().and(algaeButtonLayerTrigger.negate());
    private final Trigger sweepTrigger = driveController.x().and(algaeButtonLayerTrigger.negate());
    private final Trigger autoClimbTrigger = driveController.start();
    private final Trigger climbSequenceTrigger = buttonBoardOther.button(7);
    private final Trigger unclimbTrigger = driveController.back();
    private final Trigger climbGrabPosTrigger = buttonBoardOther.button(6).debounce(0.5, DebounceType.kRising);
    private final Trigger climbHalfwayTrigger = buttonBoardOther.button(2);
    private final Trigger algaeAndCoralToggle = algaeButtonLayerTrigger; // adds algae pickup to coral auto sequence

    private final Trigger algaeTrigger = driveController.y().and(algaeButtonLayerTrigger);
    private final Trigger algaeScoreTrigger = driveController.b().and(algaeButtonLayerTrigger);
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

    private final Trigger[] reefTriggers = new Trigger[12];
    private final Trigger[] levelTriggers = new Trigger[] {
        buttonBoardOther.button(9),
        buttonBoardOther.button(10),
        buttonBoardOther.button(11),
        buttonBoardOther.button(12)
    };

    private final Trigger barge1Trigger = buttonBoardOther.button(3);
    private final Trigger barge2Trigger = buttonBoardOther.button(4);
    private final Trigger barge3Trigger = buttonBoardOther.button(5);
    private final Trigger processPosTrigger = buttonBoardOther.button(8);
    // #endregion

    // #region Alerts
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
    // #endregion

    public RobotContainer() {
        AlignToReefCommands.testReefPoses();
        AlignToReefCommands.testBranchPoses();
        ApproachBargeCommands.testBargePoses();

        NTConstants.AUTO_DESCRIPTOR_ENTRY.set("");
        NTConstants.REEF_TELEOP_AUTO_ENTRY.set("A4");

        // #region Instantiate Subsystems
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
                        swerve::getPose,
                        // new ReefVisionIOArducam(),
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
                coralDetection = new CoralDetection(new CoralDetectionIOSim(), swerve::getPose);

                vision = new Vision(
                        swerve::addVisionMeasurement,
                        swerve::getPose,
                        // new ReefVisionIO() {},
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

                vision = new Vision(
                        swerve::addVisionMeasurement,
                        swerve::getPose,
                        // new ReefVisionIO() {},
                        new VisionIO() {},
                        new VisionIO() {});
                break;
        }
        // #endregion

        // #region Instantiate Commands
        teleopSwerveCommand = new TeleopSwerve(swerve, driveController);

        rumbleCommand = Commands.startEnd(
                        () -> driveController.setRumble(RumbleType.kBothRumble, ControllerConstants.CONTROLLER_RUMBLE),
                        () -> driveController.setRumble(RumbleType.kBothRumble, 0))
                .withName("Rumble");

        intakeAndReefCommand = Commands.defer(
                        () -> new AutoCoralCommand(swerve, intake, endEffector, elevator, coralDetection)
                                .andThen(Commands.defer(
                                        () -> new FullAutoCommand(
                                                NTConstants.REEF_TELEOP_AUTO_ENTRY.get(),
                                                swerve,
                                                elevator,
                                                endEffector,
                                                algaeManipulator,
                                                intake,
                                                coralDetection,
                                                vision),
                                        Set.of(swerve, elevator))),
                        Set.of(swerve, elevator))
                // .andThen(rumbleCommand.asProxy().withTimeout(ControllerConstants.SCORE_RUMBLE_TIME))
                .withName("Intake + Reef Auto");

        reefCommand = Commands.defer(
                        () -> new FullAutoCommand(
                                NTConstants.REEF_TELEOP_AUTO_ENTRY.get(),
                                swerve,
                                elevator,
                                endEffector,
                                algaeManipulator,
                                intake,
                                coralDetection,
                                vision),
                        Set.of(swerve, elevator))
                // .andThen(rumbleCommand.asProxy().withTimeout(ControllerConstants.SCORE_RUMBLE_TIME))
                .withName("Reef Auto");

        algaeCommand = Commands.defer(
                        () -> new RemoveAlgaeCommand(
                                NTConstants.REEF_TELEOP_AUTO_ENTRY.get(), swerve, elevator, algaeManipulator),
                        Set.of(swerve, elevator))
                .withName("Algae Auto");

        bargeCommand = Commands.defer(
                        () -> new ScoreInBargeCommand(
                                NTConstants.ALGAE_SCORE_ENTRY.get().charAt(0), swerve, elevator, algaeManipulator),
                        Set.of(swerve, elevator))
                .withName("Barge Auto");

        lollipopCommand = Commands.defer(
                        () -> LollipopCommands.lollipopCommand(swerve, algaeManipulator, elevator),
                        Set.of(swerve, elevator))
                .withName("Lollipop Auto");

        processCommand = Commands.defer(
                        () -> new ProcessCommand(swerve, elevator, algaeManipulator), Set.of(swerve, elevator))
                .withName("Process");

        ApproachReefCommand approach = new ApproachReefCommand(0, 1, swerve, elevator, vision);
        SmartDashboard.putData("Track L3", elevator.trackL3Command(approach::getDistanceToTarget)); // for debug
        // SmartDashboard.putData("Advanced Align", AlignToReefCommands.advancedAlignToReef(1, 1, swerve, vision));
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

        // adjust coral when elevator is being raised
        Trigger aboveAdjustHeightTrigger =
                new Trigger(() -> elevator.getHeight().gt(EndEffectorConstants.CORAL_ADJUST_HEIGHT)).debounce(0.1);
        aboveAdjustHeightTrigger.onTrue(endEffector.adjustCoralCommand());
        aboveAdjustHeightTrigger.onFalse(endEffector.unAdjustCoralCommand());

        autoCoralCommand = new AutoCoralCommand(swerve, intake, endEffector, elevator, coralDetection);

        climbSequence = new ClimbSequence(swerve, climber);
        // #endregion

        SmartDashboard.putData("Reef Auto", intakeAndReefCommand);
        SmartDashboard.putData("Algae Auto", algaeCommand);
        SmartDashboard.putData("Barge Auto", bargeCommand);
        SmartDashboard.putData("Nearest Lollipop", lollipopCommand);
        SmartDashboard.putData("Process", processCommand);

        SmartDashboard.putData(
                "Coral Search",
                Commands.defer(
                        () -> new FullAutoCommand(
                                "S1", swerve, elevator, endEffector, algaeManipulator, intake, coralDetection, vision),
                        Set.of(swerve, elevator)));

        configureBindings();
    }

    private void configureBindings() {
        // speedUpTrigger.whileTrue(teleopSwerveCommand.speedUpCommand());
        // slowDownTrigger.whileTrue(teleopSwerveCommand.slowDownCommand());

        elevatorUpTrigger.whileTrue(elevator.elevatorUpCommand());
        elevatorDownTrigger.whileTrue(elevator.elevatorDownCommand());
        elevatorIntakeTrigger.whileTrue(elevator.goToIntakePosCommand(true));
        // elevatorTrigger.whileTrue(elevatorCommand);

        intakeTrigger.toggleOnTrue(intake.deployCommand(true)
                .andThen(endEffector.runCommand(EndEffectorConstants.INTAKE_SPEED))
                .alongWith(intake.intakeCommand())
                .until(endEffector.coralDetectedTrigger)
                .finallyDo(() -> intake.setGoal(IntakeConstants.STOW_POS)));
        shootTrigger.whileTrue(
                intake.intakeCommand().alongWith(endEffector.runCommand(EndEffectorConstants.INTAKE_SPEED)));
        reverseIntakeTrigger.whileTrue(endEffector
                .runCommand(EndEffectorConstants.INTAKE_SPEED.unaryMinus())
                .alongWith(intake.ejectCommand()));

        toggleIntakeDeployTrigger.onTrue(Commands.defer(() -> intake.toggleCommand(false), Set.of(intake))
                .alongWith(elevator.goToIntakePosCommand(true)));
        elevatorZeroTrigger.onTrue(elevator.zeroEncoderCommand());
        autoCoralTrigger.whileTrue(autoCoralCommand);

        intakeAndReefTrigger.whileTrue(intakeAndReefCommand);
        reefTrigger.whileTrue(reefCommand);
        algaeTrigger.whileTrue(algaeCommand);
        algaeScoreTrigger.whileTrue(Commands.either(processCommand, bargeCommand, () -> NTConstants.ALGAE_SCORE_ENTRY
                .get()
                .equals("P")));
        sweepTrigger.whileTrue(sweepCommand);
        lollipopTrigger.whileTrue(lollipopCommand);

        algaeManualIntakeTrigger.whileTrue(algaeManipulator.forwardCommand());
        algaeManualEjectTrigger.whileTrue(algaeManipulator.reverseCommand());

        algaeAndCoralToggle.whileTrue(setAlgaeCommand());

        autoClimbTrigger.whileTrue(climber.autoClimbCommand());
        climbSequenceTrigger.onTrue(climber.autoClimbCommand());
        unclimbTrigger.whileTrue(climber.reverseCommand());
        climbGrabPosTrigger.onTrue(climber.goToGrabPosCommand().beforeStarting(intake.deployCommand(true)));
        climbHalfwayTrigger.whileTrue(climber.halfwayCommand());

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

        barge1Trigger.onTrue(Commands.run(() -> setAlgaeScoreDescriptor("1")));
        barge2Trigger.onTrue(Commands.run(() -> setAlgaeScoreDescriptor("2")));
        barge3Trigger.onTrue(Commands.run(() -> setAlgaeScoreDescriptor("3")));
        processPosTrigger.onTrue(Commands.run(() -> setAlgaeScoreDescriptor("P")));

        coralDetection.hasTarget.whileTrue(rumbleCommand);
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
    // #region Auto Descriptor Setters
    private void setTeleopAutoDescriptorLetter(char letter) {
        String descriptor = NTConstants.REEF_TELEOP_AUTO_ENTRY.get();
        NTConstants.REEF_TELEOP_AUTO_ENTRY.set(letter + descriptor.substring(1));
    }

    private void setTeleopAutoDescriptorLevel(int level) {
        String descriptor = NTConstants.REEF_TELEOP_AUTO_ENTRY.get();
        NTConstants.REEF_TELEOP_AUTO_ENTRY.set(descriptor.substring(0, 1) + level);
    }

    private void setAlgaeScoreDescriptor(String pos) {
        NTConstants.ALGAE_SCORE_ENTRY.set(pos);
    }

    private void setAlgaePickupDescriptor(boolean doAlgae) {
        String descriptor = NTConstants.REEF_TELEOP_AUTO_ENTRY.get();
        NTConstants.REEF_TELEOP_AUTO_ENTRY.set(descriptor.substring(0, 2) + (doAlgae ? "A" : ""));
    }

    private Command setAlgaeCommand() {
        return Commands.startEnd(() -> setAlgaePickupDescriptor(true), () -> setAlgaePickupDescriptor(false))
                .withName("Toggle Algae");
    }
    // #endregion

    public Command getAutonomousCommand() {
        return new FullAutoCommand(
                NTConstants.AUTO_DESCRIPTOR_ENTRY.get(),
                swerve,
                elevator,
                endEffector,
                algaeManipulator,
                intake,
                coralDetection,
                vision);
    }
}
