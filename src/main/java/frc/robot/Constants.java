// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.ClosedLoopGeneralConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.DigitalInputsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.MountPoseConfigs;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.Slot2Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.S1CloseStateValue;
import com.ctre.phoenix6.signals.S2CloseStateValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.ClosedLoopOutputType;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.DriveMotorArrangement;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.SteerFeedbackType;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.SteerMotorArrangement;
import com.ctre.phoenix6.swerve.SwerveModuleConstantsFactory;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Frequency;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.units.measure.MomentOfInertia;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.util.ControlConstants;
import java.util.Map;
import java.util.function.Supplier;
import org.littletonrobotics.junction.networktables.LoggedNetworkString;

/** Add your docs here. */
public class Constants {
    public static final Mode SIM_MODE = Mode.SIM;
    public static final Mode CURRENT_MODE = RobotBase.isReal() ? Mode.REAL : SIM_MODE;

    public static enum Mode {
        /** Running on a real robot. */
        REAL,

        /** Running a physics simulator. */
        SIM,

        /** Replaying from a log file. */
        REPLAY
    }

    public static final CANBus CAN_FD_BUS = new CANBus("Bobby");
    public static final CANBus CAN_RIO_BUS = new CANBus("rio");

    public static final Time SIM_LOOP_PERIOD = Milliseconds.of(20);

    public static final NetworkTableInstance INST = NetworkTableInstance.getDefault();

    public static final Voltage LOW_BATTERY_VOLTAGE = Volts.of(12.4);

    public static class Dimensions { // unfinished
        public static final Distance BUMPER_THICKNESS = Inches.of(3.2);
        public static final Distance FRAME_SIZE = Inches.of(29);
        public static final Distance ROBOT_SIZE = FRAME_SIZE.plus(BUMPER_THICKNESS.times(2));
    }

    public static class ControllerConstants {
        public static final double CONTROLLER_DEADBAND = 0.225;
        public static final double CONTROLLER_RUMBLE = 0.3;
        public static final Time SCORE_RUMBLE_TIME = Seconds.of(1);

        public static final LinearVelocity DEFAULT_DRIVE_SPEED = MetersPerSecond.of(3.2);
        public static final AngularVelocity DEFAULT_ROT_SPEED = RotationsPerSecond.of(1.25);

        public static final LinearVelocity FAST_DRIVE_SPEED = MetersPerSecond.of(4.3);
        public static final AngularVelocity FAST_ROT_SPEED = RotationsPerSecond.of(4);

        public static final LinearVelocity SLOW_DRIVE_SPEED = MetersPerSecond.of(1.5);
        public static final AngularVelocity SLOW_ROT_SPEED = RotationsPerSecond.of(0.5);

        public static final LinearAcceleration MAX_TELEOP_ACCEL = MetersPerSecondPerSecond.of(10);
    }

    public static class SwerveConstants {
        public static final AngularVelocity MAX_MODULE_ROT_SPEED = RotationsPerSecond.of(5);

        // distance between modules on same side (front to back or left to right)
        private static final Distance MODULE_DISTANCE = Inches.of(23.75);
        public static final Distance DRIVE_BASE_RADIUS =
                Meters.of(Math.sqrt(2 * MODULE_DISTANCE.in(Meters) * MODULE_DISTANCE.in(Meters)));

        private static final Slot0Configs STEER_GAINS = new Slot0Configs()
                .withKP(100)
                .withKI(5)
                .withKD(0.5)
                .withKS(0.1)
                .withKV(2.66)
                .withKA(0.0)
                .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign);

        private static final Slot0Configs DRIVE_GAINS = new Slot0Configs()
                .withKP(1.3)
                .withKI(0.01)
                .withKD(0.0)
                .withKS(0.13) // 0.11828, 0.16033, 0.12378
                .withKV(0.127) // 0.12867, 0.12604, 0.12584, 0.12882
                .withKA(0.02); // 0.016755, 0.041694, 0.0088714, 0.019146

        private static final ClosedLoopOutputType STEER_CLOSED_LOOP_OUTPUT = ClosedLoopOutputType.Voltage;
        private static final ClosedLoopOutputType DRIVE_CLOSED_LOOP_OUTPUT = ClosedLoopOutputType.Voltage;

        private static final DriveMotorArrangement DRIVE_MOTOR_TYPE = DriveMotorArrangement.TalonFX_Integrated;
        private static final SteerMotorArrangement STEER_MOTOR_TYPE = SteerMotorArrangement.TalonFX_Integrated;

        private static final SteerFeedbackType STEER_FEEDBACK_TYPE = SteerFeedbackType.FusedCANcoder;

        private static final Current SLIP_CURRENT = Amps.of(80.0); // NEEDS TUNING

        private static final TalonFXConfiguration DRIVE_CONFIGS = new TalonFXConfiguration()
                .withCurrentLimits(new CurrentLimitsConfigs()
                        .withStatorCurrentLimit(Amps.of(120))
                        .withStatorCurrentLimitEnable(true))
                .withMotorOutput(new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Brake));

        private static final TalonFXConfiguration STEER_CONFIGS = new TalonFXConfiguration()
                .withCurrentLimits(new CurrentLimitsConfigs()
                        .withStatorCurrentLimit(Amps.of(60))
                        .withStatorCurrentLimitEnable(true))
                .withMotorOutput(new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Brake));

        private static final CANcoderConfiguration ENCODER_CONFIGS = new CANcoderConfiguration();

        public static final Pigeon2Configuration PIGEON_CONFIGS =
                new Pigeon2Configuration().withMountPose(new MountPoseConfigs().withMountPoseRoll(Degrees.of(180)));

        public static final LinearVelocity SPEED_AT_12V = MetersPerSecond.of(4.44); // maybe needs tuning

        // Every 1 rotation of the azimuth results in kCoupleRatio drive motor turns;
        private static final double COUPLE_RATIO = 3.5714285714285716;

        private static final double DRIVE_GEAR_RATIO = 6.746031746031747;
        private static final double STEER_GEAR_RATIO = 21.428571428571427;
        private static final Distance WHEEL_RADIUS = Inches.of(1.875);

        public static final int PIGEON_ID = 1;

        // SIMULATION inertia
        private static final MomentOfInertia STEER_INERTIA = KilogramSquareMeters.of(0.01);
        private static final MomentOfInertia DRIVE_INERTIA = KilogramSquareMeters.of(0.01);
        // SIMULATION voltage necessary to overcome friction
        private static final Voltage STEER_FRICTION_VOLTAGE = Volts.of(0.2);
        private static final Voltage DRIVE_FRICTION_VOLTAGE = Volts.of(0.2);

        public static final SwerveDrivetrainConstants DRIVETRAIN_CONSTANTS = new SwerveDrivetrainConstants()
                .withCANBusName(CAN_FD_BUS.getName())
                .withPigeon2Id(PIGEON_ID)
                .withPigeon2Configs(PIGEON_CONFIGS);

        private static final SwerveModuleConstantsFactory<
                        TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
                CONSTANT_CREATOR = new SwerveModuleConstantsFactory<
                                TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>()
                        .withDriveMotorGearRatio(DRIVE_GEAR_RATIO)
                        .withSteerMotorGearRatio(STEER_GEAR_RATIO)
                        .withCouplingGearRatio(COUPLE_RATIO)
                        .withWheelRadius(WHEEL_RADIUS)
                        .withSteerMotorGains(STEER_GAINS)
                        .withDriveMotorGains(DRIVE_GAINS)
                        .withSteerMotorClosedLoopOutput(STEER_CLOSED_LOOP_OUTPUT)
                        .withDriveMotorClosedLoopOutput(DRIVE_CLOSED_LOOP_OUTPUT)
                        .withSlipCurrent(SLIP_CURRENT)
                        .withSpeedAt12Volts(SPEED_AT_12V)
                        .withDriveMotorType(DRIVE_MOTOR_TYPE)
                        .withSteerMotorType(STEER_MOTOR_TYPE)
                        .withFeedbackSource(STEER_FEEDBACK_TYPE)
                        .withDriveMotorInitialConfigs(DRIVE_CONFIGS)
                        .withSteerMotorInitialConfigs(STEER_CONFIGS)
                        .withEncoderInitialConfigs(ENCODER_CONFIGS)
                        .withSteerInertia(STEER_INERTIA)
                        .withDriveInertia(DRIVE_INERTIA)
                        .withSteerFrictionVoltage(STEER_FRICTION_VOLTAGE)
                        .withDriveFrictionVoltage(DRIVE_FRICTION_VOLTAGE);

        public static class FrontLeft {
            private static final int DRIVE_ID = 1;
            private static final int STEER_ID = 5;
            private static final int ENCODER_ID = 1;
            private static final Angle ENCODER_OFFSET = Rotations.of(0.268798828125);
            private static final boolean STEER_INVERTED = true;
            private static final boolean ENCODER_INVERTED = false;
            private static final boolean DRIVE_INVERTED = false;

            public static final Distance X_POS = MODULE_DISTANCE.div(2);
            public static final Distance Y_POS = MODULE_DISTANCE.div(2);

            public static final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
                    MODULE_CONSTANTS = CONSTANT_CREATOR.createModuleConstants(
                            STEER_ID,
                            DRIVE_ID,
                            ENCODER_ID,
                            ENCODER_OFFSET,
                            X_POS,
                            Y_POS,
                            DRIVE_INVERTED,
                            STEER_INVERTED,
                            ENCODER_INVERTED);
        }

        public static class FrontRight {
            private static final int DRIVE_ID = 2;
            private static final int STEER_ID = 6;
            private static final int ENCODER_ID = 2;
            private static final Angle ENCODER_OFFSET = Rotations.of(-0.0693359375);
            private static final boolean STEER_INVERTED = true;
            private static final boolean ENCODER_INVERTED = false;
            private static final boolean DRIVE_INVERTED = true;

            public static final Distance X_POS = MODULE_DISTANCE.div(2);
            public static final Distance Y_POS = MODULE_DISTANCE.div(-2);

            public static final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
                    MODULE_CONSTANTS = CONSTANT_CREATOR.createModuleConstants(
                            STEER_ID,
                            DRIVE_ID,
                            ENCODER_ID,
                            ENCODER_OFFSET,
                            X_POS,
                            Y_POS,
                            DRIVE_INVERTED,
                            STEER_INVERTED,
                            ENCODER_INVERTED);
        }

        public static class BackLeft {
            private static final int DRIVE_ID = 4;
            private static final int STEER_ID = 8;
            private static final int ENCODER_ID = 4;
            private static final Angle ENCODER_OFFSET = Rotations.of(0.361083984375);
            private static final boolean STEER_INVERTED = true;
            private static final boolean ENCODER_INVERTED = false;
            private static final boolean DRIVE_INVERTED = false;

            public static final Distance X_POS = MODULE_DISTANCE.div(-2);
            public static final Distance Y_POS = MODULE_DISTANCE.div(2);

            public static final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
                    MODULE_CONSTANTS = CONSTANT_CREATOR.createModuleConstants(
                            STEER_ID,
                            DRIVE_ID,
                            ENCODER_ID,
                            ENCODER_OFFSET,
                            X_POS,
                            Y_POS,
                            DRIVE_INVERTED,
                            STEER_INVERTED,
                            ENCODER_INVERTED);
        }

        public static class BackRight {
            private static final int DRIVE_ID = 3;
            private static final int STEER_ID = 7;
            private static final int ENCODER_ID = 3;
            private static final Angle ENCODER_OFFSET = Rotations.of(-0.25830078125);
            private static final boolean STEER_INVERTED = true;
            private static final boolean ENCODER_INVERTED = false;
            private static final boolean DRIVE_INVERTED = true;

            public static final Distance X_POS = MODULE_DISTANCE.div(-2);
            public static final Distance Y_POS = MODULE_DISTANCE.div(-2);

            public static final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
                    MODULE_CONSTANTS = CONSTANT_CREATOR.createModuleConstants(
                            STEER_ID,
                            DRIVE_ID,
                            ENCODER_ID,
                            ENCODER_OFFSET,
                            X_POS,
                            Y_POS,
                            DRIVE_INVERTED,
                            STEER_INVERTED,
                            ENCODER_INVERTED);
        }

        public static final Supplier<Translation2d[]> GET_MODULE_POSITIONS = () -> new Translation2d[] {
            new Translation2d(SwerveConstants.FrontLeft.X_POS, SwerveConstants.FrontLeft.Y_POS),
            new Translation2d(SwerveConstants.FrontRight.X_POS, SwerveConstants.FrontRight.Y_POS),
            new Translation2d(SwerveConstants.BackLeft.X_POS, SwerveConstants.BackLeft.Y_POS),
            new Translation2d(SwerveConstants.BackRight.X_POS, SwerveConstants.BackRight.Y_POS),
        };

        public static final Frequency ODOMETRY_UPDATE_FREQ = Hertz.of(0); // 0 Hz = default 250 Hz for CAN FD
        public static final Matrix<N3, N1> ODOMETRY_STD_DEV = VecBuilder.fill(0.02, 0.02, 0.01);

        public static final DriveRequestType DRIVE_REQUEST_TYPE = DriveRequestType.Velocity;
        public static final SteerRequestType STEER_REQUEST_TYPE = SteerRequestType.MotionMagicExpo;

        public static final LinearVelocity LINEAR_VEL_DEADBAND = MetersPerSecond.of(0.02);
        public static final AngularVelocity ANGLULAR_VEL_DEADBAND = DegreesPerSecond.of(1);
    }

    public static class AlignConstants {
        // output: m/s, measure: m
        public static final ControlConstants SCORING_PID_TRANSLATION = new ControlConstants()
                .withPID(1.5, 0.5, 0.0)
                .withFeedforward(1, 0)
                .withTolerance(Inches.of(2).in(Meters), 0.1)
                .withProfile(2, 2);

        public static final ControlConstants ALGAE_PICK_PID_TRANSLATION = new ControlConstants(SCORING_PID_TRANSLATION)
                .withProfile(2, 6)
                .withTolerance(Inches.of(3).in(Meters));

        public static final ControlConstants ALGAE_PULL_PID_TRANSLATION = new ControlConstants(SCORING_PID_TRANSLATION)
                .withProfile(3.5, 6)
                .withTolerance(Inches.of(4).in(Meters));

        public static final ControlConstants CORAL_PULL_PID_TRANSLATION = new ControlConstants(SCORING_PID_TRANSLATION)
                .withProfile(3.5, 6)
                .withTolerance(Inches.of(8).in(Meters));

        public static final ControlConstants PROCESS_PID_TRANSLATION = new ControlConstants(SCORING_PID_TRANSLATION)
                .withTolerance(Inches.of(4).in(Meters));

        public static final ControlConstants SWEEP_PID_TRANSLATION = new ControlConstants(SCORING_PID_TRANSLATION)
                .withTolerance(Inches.of(4).in(Meters));

        public static final ControlConstants LOLLIPOP_PID_TRANSLATION = new ControlConstants(SCORING_PID_TRANSLATION)
                .withProfile(3, 7)
                .withTolerance(Inches.of(4).in(Meters));

        public static final ControlConstants CORAL_PICKUP_PID_TRANSLATION = new ControlConstants(
                        SCORING_PID_TRANSLATION)
                .withProfile(3, 6)
                .withTolerance(Inches.of(4).in(Meters));

        // output: deg/s, measure: deg
        public static final ControlConstants SCORING_PID_ANGLE =
                new ControlConstants().withPID(1.5, 0.5, 0.0).withTolerance(1.2);

        public static final ControlConstants ALGAE_PICK_PID_ANGLE =
                new ControlConstants(SCORING_PID_ANGLE).withTolerance(3);
        public static final ControlConstants ALGAE_PULL_PID_ANGLE =
                new ControlConstants(SCORING_PID_ANGLE).withTolerance(6);

        public static final ControlConstants CORAL_PULL_PID_ANGLE =
                new ControlConstants(SCORING_PID_ANGLE).withTolerance(10);

        public static final ControlConstants PROCESS_PID_ANGLE =
                new ControlConstants(SCORING_PID_ANGLE).withTolerance(4);

        public static final ControlConstants SWEEP_PID_ANGLE = new ControlConstants(SCORING_PID_ANGLE).withTolerance(5);

        public static final ControlConstants LOLLIPOP_PID_ANGLE =
                new ControlConstants(SCORING_PID_ANGLE).withTolerance(5);

        public static final ControlConstants CORAL_PICKUP_PID_ANGLE = new ControlConstants().withPID(6, 2, 0.0);

        public static final Time ALIGN_TIME = Seconds.of(0.1); // amount to wait to make sure aligned
    }

    public static class ElevatorConstants {
        // Motors
        public static final int LEAD_MOTOR_ID = 12; // fd bus
        public static final int FOLLOW_MOTOR_ID = 16; // fd bus
        public static final int ENCODER_ID = 5; // fd bus

        // Motor Configs
        public static final double GEAR_RATIO = 76.0 / 18;
        public static final Distance DRUM_RADIUS = Inches.of(1);

        public static final boolean OPPOSE_FOLLOWER = true;

        public static final CurrentLimitsConfigs CURRENT_LIMITS_CONFIGS =
                new CurrentLimitsConfigs().withStatorCurrentLimit(Amps.of(70));

        public static final MotorOutputConfigs OUTPUT_CONFIGS = new MotorOutputConfigs()
                .withInverted(InvertedValue.Clockwise_Positive)
                .withNeutralMode(NeutralModeValue.Brake);

        public static final FeedbackConfigs FEEDBACK_CONFIGS = new FeedbackConfigs()
                .withFeedbackRemoteSensorID(ENCODER_ID)
                .withFeedbackSensorSource(FeedbackSensorSourceValue.RemoteCANcoder)
                .withRotorToSensorRatio(GEAR_RATIO);

        // volts, radians
        // used for sim
        public static final Slot0Configs STAGE3_GAINS = CURRENT_MODE == SIM_MODE
                ? new Slot0Configs() // sim gains (all stages)
                        .withKP(4)
                        .withKI(1)
                        .withKD(0.01)
                        .withKV(0.3)
                        .withKA(0.0)
                        .withKS(0.01)
                        .withKG(0.54)
                : new Slot0Configs() // real gains
                        .withKP(10)
                        .withKI(3)
                        .withKD(0.1)
                        .withKV(0.48)
                        .withKA(0.017)
                        .withKS(0.3)
                        .withKG(0.0)
                        .withGravityType(GravityTypeValue.Elevator_Static);
        public static final Slot1Configs STAGE2_GAINS = new Slot1Configs() // real gains
                .withKP(12)
                .withKI(4)
                .withKD(0.15)
                .withKV(0.5)
                .withKA(0.01)
                .withKS(0.12)
                .withKG(0.36)
                .withGravityType(GravityTypeValue.Elevator_Static);
        public static final Slot2Configs STAGE1_GAINS = new Slot2Configs() // real gains
                .withKP(15)
                .withKI(7)
                .withKD(0.1)
                .withKV(0.55)
                .withKA(0.015)
                .withKS(0.15)
                .withKG(0.41)
                .withGravityType(GravityTypeValue.Elevator_Static);

        public static final MotionMagicConfigs MOTION_MAGIC_CONFIGS = new MotionMagicConfigs()
                .withMotionMagicExpo_kV(Volts.of(0.4).per(RotationsPerSecond))
                .withMotionMagicExpo_kA(Volts.of(0.4).per(RotationsPerSecondPerSecond));

        public static final TalonFXConfiguration MOTOR_CONFIGS = new TalonFXConfiguration()
                .withCurrentLimits(CURRENT_LIMITS_CONFIGS)
                .withMotorOutput(OUTPUT_CONFIGS)
                .withFeedback(FEEDBACK_CONFIGS)
                .withMotionMagic(MOTION_MAGIC_CONFIGS)
                .withSlot0(STAGE3_GAINS)
                .withSlot1(STAGE2_GAINS)
                .withSlot2(STAGE1_GAINS);

        public static final MagnetSensorConfigs ENCODER_CONFIGS = new MagnetSensorConfigs()
                .withMagnetOffset(-0.345947265625)
                .withSensorDirection(SensorDirectionValue.Clockwise_Positive);

        public static final Time AT_GOAL_DEBOUNCE_TIME = Seconds.of(0.05);

        public static final Distance TOLERANCE = Inches.of(0.5);

        // Manual control (duty cycle)
        public static final Voltage MANUAL_UP_SPEED = Volts.of(3.6);
        public static final Voltage MANUAL_DOWN_SPEED = Volts.of(-2.4);

        public static final Current STALL_CURRENT = Amps.of(60);

        // Sim constants
        public static final Mass CARRIAGE_MASS = Pounds.of(10);

        // Setpoints (from floor)
        public static final Distance MIN_HEIGHT = Inches.of(10);
        public static final Distance MAX_HEIGHT = Inches.of(81.19);
        public static final Distance L1_HEIGHT = Inches.of(29.8);
        public static final Distance L2_HEIGHT = Inches.of(38.5);
        public static final Distance L3_HEIGHT = Inches.of(53.8);
        public static final Distance L4_HEIGHT = Inches.of(77.5);
        public static final Distance INTAKE_HEIGHT = Inches.of(10.5);

        public static final Distance STAGE2_HEIGHT = Inches.of(30.54); // height when stage 2 starts being lifted
        public static final Distance STAGE1_HEIGHT = Inches.of(56.68); // height when stage 1 starts being lifted

        public static final Distance LOW_ALGAE_HEIGHT = Meters.of(0.69);
        public static final Distance HIGH_ALGAE_HEIGHT = Meters.of(1.1420);
        public static final Distance BARGE_HEIGHT = Inches.of(82.4);
        public static final Distance LOLLIPOP_HEIGHT = Inches.of(9.3);
        public static final Distance PROCESS_HEIGHT = Inches.of(9.3);

        public static final Map<Integer, Distance> ALGAE_HEIGHTS = Map.of(
                0, HIGH_ALGAE_HEIGHT,
                1, LOW_ALGAE_HEIGHT,
                2, HIGH_ALGAE_HEIGHT,
                3, LOW_ALGAE_HEIGHT,
                4, HIGH_ALGAE_HEIGHT,
                5, LOW_ALGAE_HEIGHT);

        public static final Angle SHOOT_ANGLE = Degrees.of(30.73124803);

        public static final Distance MAX_SHOOT_DISTANCE =
                MAX_HEIGHT.minus(L4_HEIGHT).div(Math.tan(SHOOT_ANGLE.in(Radians)));
    }

    public static class EndEffectorConstants {
        // Motors
        public static final int MOTOR_LEFT_ID = 30;
        public static final int MOTOR_RIGHT_ID = 31;

        public static final MotorOutputConfigs MOTOR_LEFT_CONFIGS =
                new MotorOutputConfigs().withInverted(InvertedValue.CounterClockwise_Positive);
        public static final MotorOutputConfigs MOTOR_RIGHT_CONFIGS =
                new MotorOutputConfigs().withInverted(InvertedValue.Clockwise_Positive);
        public static final CurrentLimitsConfigs CURRENT_LIMITS_CONFIGS =
                new CurrentLimitsConfigs().withSupplyCurrentLimit(Amps.of(30)).withStatorCurrentLimitEnable(false);

        // Coral detection
        public static final int LIDAR_ID = 9; // DIO
        public static final Current CORAL_DETECTION_CURRENT = Amps.of(45);
        public static final Time CORAL_SHOOT_TIME = Seconds.of(0.3);
        public static final Time L1_SHOOT_TIME = Seconds.of(0.5);

        // Speed (voltage)
        public static final Voltage INTAKE_SPEED = Volts.of(4.5);
        public static final Voltage SLOW_INTAKE_SPEED = Volts.of(3.6);
        public static final Voltage SCORE_SPEED = Volts.of(1.8);
        public static final Voltage FAST_TROUGH_SPEED = Volts.of(3.6);
        public static final Voltage SLOW_TROUGH_SPEED = Volts.of(2);
        public static final Voltage ADJUST_SPEED = Volts.of(1);

        public static final Time COOLER_INTAKE_CYCLE = Seconds.of(0.1);
        public static final AngularVelocity MIN_VEL = RotationsPerSecond.of(2);
        public static final Time STALL_TIME = Seconds.of(0.5);
        public static final Time ADJUST_TIME = Seconds.of(0.15);
        public static final Time UNADJUST_TIME = Seconds.of(0.1);

        public static final Distance CORAL_ADJUST_HEIGHT = Meters.of(1.3);
    }

    public static class IntakeConstants {
        public static final int INTAKE_MOTOR_ID = 33;
        public static final int DEPLOY_MOTOR_ID = 22;
        public static final int ALIGN_MOTOR_ID = 11;

        public static final int INTAKE_CANDI_ID = 1;

        public static final int ENCODER_ID = 21;

        public static final Distance INTAKE_EXTENSION = Inches.of(10.5);
        // public static final Distance DEPLOY_CLEARANCE = Inches.of(18);
        // public static final Distance SLOWDOWN_START_DISTANCE = Inches.of(36);
        // public static final Distance SLOWDOWN_STOP_DISTANCE = Inches.of(12);
        public static final Distance DISTANCE_TO_KEEP_FROM_WALL = Inches.of(16);
        public static final Distance SLOWDOWN_DISTANCE = Inches.of(48);
        public static final Angle ANGLE_TO_FACE_WALL = Degrees.of(90); // limit vel if the bot is <=45 deg to wall
        public static final Distance START_DISTANCE = Inches.of(65);

        public static final MotorOutputConfigs INTAKE_MOTOR_OUTPUT_CONFIGS = new MotorOutputConfigs()
                .withInverted(InvertedValue.CounterClockwise_Positive)
                .withNeutralMode(NeutralModeValue.Brake);
        public static final CurrentLimitsConfigs INTAKE_CURRENT_LIMITS_CONFIGS = new CurrentLimitsConfigs()
                .withSupplyCurrentLimit(Amps.of(30))
                .withSupplyCurrentLowerLimit(Amps.of(30))
                .withSupplyCurrentLowerTime(Seconds.of(1))
                .withSupplyCurrentLimitEnable(true)
                .withStatorCurrentLimitEnable(false);
        public static final TalonFXConfiguration INTAKE_MOTOR_CONFIGS = new TalonFXConfiguration()
                .withMotorOutput(INTAKE_MOTOR_OUTPUT_CONFIGS)
                .withCurrentLimits(INTAKE_CURRENT_LIMITS_CONFIGS);

        public static final MotorOutputConfigs DEPLOY_MOTOR_OUTPUT_CONFIGS = new MotorOutputConfigs()
                .withInverted(InvertedValue.CounterClockwise_Positive)
                .withNeutralMode(NeutralModeValue.Brake);
        public static final CurrentLimitsConfigs DEPLOY_CURRENT_LIMITS_CONFIGS = new CurrentLimitsConfigs()
                .withSupplyCurrentLimit(Amps.of(30))
                .withSupplyCurrentLowerLimit(Amps.of(15))
                .withSupplyCurrentLowerTime(Seconds.of(1))
                .withSupplyCurrentLimitEnable(true)
                .withStatorCurrentLimitEnable(false);
        public static final FeedbackConfigs DEPLOY_FEEDBACK_CONFIGS = new FeedbackConfigs()
                .withFeedbackRemoteSensorID(ENCODER_ID)
                .withFeedbackSensorSource(FeedbackSensorSourceValue.RemoteCANcoder)
                .withRotorToSensorRatio(20 * 50 / 24.0); // 20:1 gearbox and 50:24 gears
        public static final Slot0Configs DEPLOY_PID = new Slot0Configs()
                .withKP(3)
                .withKI(0.1)
                .withKD(1)
                .withKS(0.1)
                .withKV(8)
                .withKA(0)
                .withKG(0.3)
                .withGravityType(GravityTypeValue.Arm_Cosine)
                .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign);
        public static final Slot1Configs EMERGENCY_PID = new Slot1Configs()
                .withKP(3)
                .withKI(0.1)
                .withKD(1)
                .withKS(0.1)
                .withKV(8)
                .withKA(0)
                .withKG(0.3)
                .withGravityType(GravityTypeValue.Arm_Cosine)
                .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign);
        public static final MotionMagicConfigs DEPLOY_MOTION_MAGIC_CONFIGS = new MotionMagicConfigs()
                .withMotionMagicExpo_kV(Volts.of(6).per(RotationsPerSecond))
                .withMotionMagicExpo_kA(Volts.of(15).per(RotationsPerSecondPerSecond));
        public static final MotionMagicConfigs STOW_MOTION_MAGIC_CONFIGS = new MotionMagicConfigs()
                .withMotionMagicExpo_kV(Volts.of(6).per(RotationsPerSecond))
                .withMotionMagicExpo_kA(Volts.of(6).per(RotationsPerSecondPerSecond));
        public static final ClosedLoopGeneralConfigs DEPLOY_CLOSED_LOOP_GENERAL_CONFIGS =
                new ClosedLoopGeneralConfigs().withContinuousWrap(true);
        public static final TalonFXConfiguration DEPLOY_MOTOR_CONFIGS = new TalonFXConfiguration()
                .withMotorOutput(DEPLOY_MOTOR_OUTPUT_CONFIGS)
                .withCurrentLimits(DEPLOY_CURRENT_LIMITS_CONFIGS)
                .withFeedback(DEPLOY_FEEDBACK_CONFIGS)
                .withSlot0(DEPLOY_PID)
                .withSlot1(EMERGENCY_PID)
                .withMotionMagic(DEPLOY_MOTION_MAGIC_CONFIGS)
                .withClosedLoopGeneral(DEPLOY_CLOSED_LOOP_GENERAL_CONFIGS);

        public static final MotorOutputConfigs ALIGN_MOTOR_OUTPUT_CONFIGS = new MotorOutputConfigs()
                .withInverted(InvertedValue.CounterClockwise_Positive)
                .withNeutralMode(NeutralModeValue.Brake);
        public static final CurrentLimitsConfigs ALIGN_CURRENT_LIMITS_CONFIGS = new CurrentLimitsConfigs()
                .withSupplyCurrentLimit(Amps.of(40))
                .withSupplyCurrentLowerLimit(Amps.of(30))
                .withSupplyCurrentLowerTime(Seconds.of(1))
                .withSupplyCurrentLimitEnable(true)
                .withStatorCurrentLimitEnable(false);
        public static final TalonFXConfiguration ALIGN_MOTOR_CONFIGS = new TalonFXConfiguration()
                .withMotorOutput(ALIGN_MOTOR_OUTPUT_CONFIGS)
                .withCurrentLimits(ALIGN_CURRENT_LIMITS_CONFIGS);

        public static final MagnetSensorConfigs ENCODER_CONFIGS = new MagnetSensorConfigs()
                .withMagnetOffset(0.4111328125)
                .withSensorDirection(SensorDirectionValue.Clockwise_Positive);

        // S1 = align, S2 = beam break
        public static final DigitalInputsConfigs CANDI_CONFIGS = new DigitalInputsConfigs()
                .withS1CloseState(S1CloseStateValue.CloseWhenLow)
                .withS2CloseState(S2CloseStateValue.CloseWhenNotLow);

        public static final Voltage INTAKE_SPEED = Volts.of(8);
        public static final Voltage SLOW_INTAKE_SPEED = Volts.of(3);
        public static final Voltage EJECT_SPEED = Volts.of(-6);
        public static final Voltage DEPLOY_SPEED = Volts.of(12);
        public static final Voltage RETRACT_SPEED = Volts.of(-12);
        public static final Voltage ALIGN_SPEED = Volts.of(2.4);

        public static final Angle DEPLOY_POS = Degrees.of(0);
        public static final Angle STOW_POS = Degrees.of(70);
        // public static final Angle SLOWING_THRESHOLD = Degrees.of(35);

        public static final Angle DEPLOY_TOLERANCE = Degrees.of(40);
        public static final Angle STOW_TOLERANCE = Degrees.of(10);

        public static final Current CORAL_DETECTION_CURRENT = Amps.of(38);

        public static final LinearVelocity PICKUP_SPEED = MetersPerSecond.of(3);
        public static final AngularVelocity CORAL_SCAN_SPEED = DegreesPerSecond.of(50);

        public static final Time CORAL_TIMEOUT = Seconds.of(4);
        public static final Time INTAKE_STARTUP_TIME = Seconds.of(0.4);
        public static final Time JAM_TIME = Seconds.of(1);
        public static final Time UNJAM_TIME = Seconds.of(0.15);
        // time from coral detected in aligner to stop before end effector
        public static final Time ALIGNER_INTAKE_TIME = Seconds.of(0.05);
        public static final AngularVelocity MAX_STALL_VELOCITY = RotationsPerSecond.of(4);
        public static final Current MIN_STALL_CURRENT = Amps.of(5);

        public static final Distance CORAL_ON_WALL_THRESHOLD = Inches.of(12);
        public static final Distance MAX_CORAL_DISTANCE = Feet.of(20);

        // radians -> volts
        public static final ControlConstants DEPLOY_PID_SIM = new ControlConstants()
                .withPID(0.1, 0, 0)
                .withTolerance(Degrees.of(10).in(Radians))
                .withFeedforward(6, 0)
                .withProfile(Math.PI / 2, Math.PI);
    }

    public static class AlgaeManipulatorConstants {
        // Motor
        public static final int MOTOR_ID = 32;

        public static final MotorOutputConfigs MOTOR_CONFIGS = new MotorOutputConfigs()
                .withInverted(InvertedValue.CounterClockwise_Positive)
                .withNeutralMode(NeutralModeValue.Brake);
        public static final CurrentLimitsConfigs CURRENT_LIMITS_CONFIGS = new CurrentLimitsConfigs()
                .withSupplyCurrentLimit(Amps.of(50))
                .withSupplyCurrentLowerLimit(Amps.of(29.5))
                .withSupplyCurrentLowerTime(Seconds.of(1))
                .withSupplyCurrentLimitEnable(true)
                .withStatorCurrentLimitEnable(false);

        // Speed (voltage)
        public static final Voltage INTAKE_SPEED = Volts.of(12);
        public static final Voltage HOLD_SPEED = Volts.of(6);
        public static final Voltage EJECT_SPEED = Volts.of(-12);
        public static final Voltage FLIP_UP_SPEED = Volts.of(-10);
        public static final Voltage HOLD_UP_SPEED = Volts.of(-3.5);

        public static final AngularVelocity MIN_VEL = RotationsPerSecond.of(4);
        public static final Voltage STALL_VOLTAGE = Volts.of(0.1);

        public static final Time FLIP_UP_TIME = Seconds.of(3);
        public static final Time HOLD_TIME = Seconds.of(3);
        public static final Time HOLD_CYCLE_ON = Seconds.of(1);
        public static final Time HOLD_CYCLE_OFF = Seconds.of(4);
        public static final Time SHOOT_TIME = Seconds.of(0.15);
    }

    public static class ClimberConstants {
        public static final int CLIMB_MOTOR_ID = 14; // fd bus
        public static final int CLIMB_ENCODER_ID = 6; // fd bus
        public static final CurrentLimitsConfigs CURRENT_LIMITS_CONFIGS =
                new CurrentLimitsConfigs().withStatorCurrentLimit(Amps.of(40));

        public static final MotorOutputConfigs OUTPUT_CONFIGS = new MotorOutputConfigs()
                .withInverted(InvertedValue.Clockwise_Positive)
                .withNeutralMode(NeutralModeValue.Brake);

        public static final double GEAR_RATIO = (46.0 / 26) * (54.0 / 20) * 100;

        public static final FeedbackConfigs FEEDBACK_CONFIGS = new FeedbackConfigs()
                .withFeedbackRemoteSensorID(CLIMB_ENCODER_ID)
                .withFeedbackSensorSource(FeedbackSensorSourceValue.RemoteCANcoder)
                .withRotorToSensorRatio(GEAR_RATIO);

        public static final Angle GRAB_ANGLE = Rotations.of(-0.02);
        public static final Angle MAX_CLIMB_ANGLE = Rotations.of(0.31);
        public static final Angle RESET_ANGLE = Rotations.of(0.315);
        public static final Angle SLOW_ANGLE = Rotations.of(0.28);

        public static final TalonFXConfiguration CLIMB_CONFIGS = new TalonFXConfiguration()
                .withMotorOutput(OUTPUT_CONFIGS)
                .withCurrentLimits(CURRENT_LIMITS_CONFIGS)
                .withFeedback(FEEDBACK_CONFIGS);

        public static final MagnetSensorConfigs ENCODER_CONFIGS =
                new MagnetSensorConfigs().withMagnetOffset(0.230224609375).withAbsoluteSensorDiscontinuityPoint(0.5);

        public static final Voltage CLIMB_SPEED = Volts.of(12);
        public static final Voltage SLOW_CLIMB_SPEED = Volts.of(4);
        public static final Voltage REVERSE_SPEED = Volts.of(-12);

        public static final int GRAB_MOTOR_ID = 15;
        public static final Voltage GRAB_SPEED = Volts.of(6);
        public static final Voltage RELEASE_SPEED = Volts.of(-3);
        public static final MotorOutputConfigs GRAB_CONFIGS = new MotorOutputConfigs()
                .withInverted(InvertedValue.Clockwise_Positive)
                .withNeutralMode(NeutralModeValue.Brake);

        public static final AngularVelocity MAX_STALL_VELOCITY = RotationsPerSecond.of(1);
        public static final Voltage MIN_STALL_VOLTS = Volts.of(1);
        public static final AngularVelocity SWERVE_TURN_SPEED = DegreesPerSecond.of(120);
        private static final Angle SWERVE_TURN_AMOUNT = Degrees.of(45);
        public static final Time SWERVE_TURN_TIME =
                Seconds.of(SWERVE_TURN_AMOUNT.divideRatio(SWERVE_TURN_SPEED).in(Seconds));
    }

    public static class VisionConstants {
        // Standard deviation baselines for 1 meter distance to single tag
        public static final double LINEAR_STD_DEV_BASELINE = 0.04; // Meters
        public static final double ANGULAR_STD_DEV_BASELINE = 0.4; // Radians

        public static final String FRONT_LEFT_CAM_NAME = "Arducam_OV9281_FL01";
        public static final String FRONT_RIGHT_CAM_NAME = "Arducam_OV9281_FR01";

        public static final double MAX_AMBIGUITY = 0.3;
        public static final Distance MAX_Z_ERROR = Meters.of(0.75);

        public static final AprilTagFieldLayout APRIL_TAGS =
                AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeAndyMark);

        // Transforms from robot to cameras, (x forward, y left, z up), (roll, pitch,
        // yaw)
        public static final Transform3d FRONT_LEFT_CAM_POS = new Transform3d(
                new Translation3d(Inches.of(29.0 / 2 - 6.487), Inches.of(29.0 / 2 - 2.25), Inches.of(7.74638805)),
                new Rotation3d(Degrees.of(-1.7), Degrees.of(-24), Degrees.of(-30.65)));

        public static final Transform3d FRONT_RIGHT_CAM_POS = new Transform3d(
                new Translation3d(Inches.of(29.0 / 2 - 6.487), Inches.of(-29.0 / 2 + 2.25), Inches.of(7.74638805)),
                new Rotation3d(Degrees.of(1.7), Degrees.of(-24), Degrees.of(30.65)));

        public static final String CORAL_CAM_NAME = "Coral Camera";
        public static final Transform3d CORAL_CAM_POS = new Transform3d(
                new Translation3d(Inches.of(0.39), Inches.of(9.98), Inches.of(34.31)),
                new Rotation3d(
                        Degrees.of(0), Degrees.of(22.5), Degrees.of(-170))); // used to be 172.6 Degrees.of(169)));

        public static final int REEF_VISION_CANDI_ID = 2; // fd bus
        public static final Angle VERTICAL_FOV = Degrees.of(46);
        public static final Distance MIN_DISTANCE = Millimeters.of(170);
        public static final Distance MAX_DISTANCE = Millimeters.of(1000);
        public static final Distance MIN_HEIGHT_FOR_ACCURACY = Inches.of(70); // of elevator
        public static final Distance MAX_DISTANCE_TO_BRANCH =
                Inches.of(12); // max distance from detected branch to ideal branch pos
        public static final Translation2d TOF_CAM_POS = new Translation2d(Inches.of(10.5), Inches.zero());
    }

    public static class FieldConstants {
        public static final Map<Character, Pair<Integer, Integer>> LETTER_TO_SIDE_AND_RELATIVE = Map.ofEntries(
                Map.entry(Character.valueOf('A'), new Pair<Integer, Integer>(0, 1)),
                Map.entry(Character.valueOf('B'), new Pair<Integer, Integer>(0, -1)),
                Map.entry(Character.valueOf('C'), new Pair<Integer, Integer>(5, 1)),
                Map.entry(Character.valueOf('D'), new Pair<Integer, Integer>(5, -1)),
                Map.entry(Character.valueOf('E'), new Pair<Integer, Integer>(4, 1)),
                Map.entry(Character.valueOf('F'), new Pair<Integer, Integer>(4, -1)),
                Map.entry(Character.valueOf('G'), new Pair<Integer, Integer>(3, 1)),
                Map.entry(Character.valueOf('H'), new Pair<Integer, Integer>(3, -1)),
                Map.entry(Character.valueOf('I'), new Pair<Integer, Integer>(2, 1)),
                Map.entry(Character.valueOf('J'), new Pair<Integer, Integer>(2, -1)),
                Map.entry(Character.valueOf('K'), new Pair<Integer, Integer>(1, 1)),
                Map.entry(Character.valueOf('L'), new Pair<Integer, Integer>(1, -1)));

        public static final Pose2d LEFT_CORAL_STATION =
                new Pose2d(Meters.of(1.16), Meters.of(7.013), Rotation2d.fromDegrees(125));

        public static final Pose2d RIGHT_CORAL_STATION =
                new Pose2d(Meters.of(1.16), Meters.of(1.02), Rotation2d.fromDegrees(-125));

        public static final Pose2d LEFT_CORAL_SEARCH_POSE =
                new Pose2d(Meters.of(3.32), Meters.of(5.6), Rotation2d.fromDegrees(-35));
        public static final Pose2d RIGHT_CORAL_SEARCH_POSE =
                new Pose2d(Meters.of(3.32), Meters.of(2.5), Rotation2d.fromDegrees(35));

        public static final Pose2d PROCESSOR = new Pose2d(
                VisionConstants.APRIL_TAGS.getTagPose(16).get().toPose2d().getMeasureX(),
                PathConstants.DISTANCE_TO_PROCESSOR,
                Rotation2d.kCW_90deg);

        // Pose at midpoint between tags 18 and 21 (which are opposite on blue reef)
        public static final Translation2d REEF_CENTER_BLUE = VisionConstants.APRIL_TAGS
                .getTagPose(18)
                .get()
                .toPose2d()
                .getTranslation()
                .plus(VisionConstants.APRIL_TAGS.getTagPose(21).get().toPose2d().getTranslation())
                .div(2);

        // Pose at midpoint between tags 10 and 7 (which are opposite on red reef)
        public static final Translation2d REEF_CENTER_RED = VisionConstants.APRIL_TAGS
                .getTagPose(10)
                .get()
                .toPose2d()
                .getTranslation()
                .plus(VisionConstants.APRIL_TAGS.getTagPose(7).get().toPose2d().getTranslation())
                .div(2);

        // Distance from center of robot to center of reef
        // Found by taking distance from tag 18 to center and adding offset from reef
        public static final Distance REEF_APOTHEM = Meters.of(VisionConstants.APRIL_TAGS
                        .getTagPose(18)
                        .get()
                        .toPose2d()
                        .getTranslation()
                        .getDistance(REEF_CENTER_BLUE))
                .plus(PathConstants.DISTANCE_TO_REEF);

        // translation to move from centered on a side to scoring position for the left
        // branch
        public static final Translation2d CENTERED_TO_LEFT_BRANCH =
                new Translation2d(Meters.of(0), Inches.of(12.94 / 2));

        private static final Rotation2d BARGE_SHOOT_ROTATION = Rotation2d.fromDegrees(-20);
        public static final Distance BARGE_X = Meters.of(7.5);

        public static final Map<Character, Pose2d> BARGE_POSES = Map.of(
                '3', new Pose2d(BARGE_X, Meters.of(7.4), BARGE_SHOOT_ROTATION),
                '2', new Pose2d(BARGE_X, Meters.of(5.9), BARGE_SHOOT_ROTATION),
                '1', new Pose2d(BARGE_X, Meters.of(4.3), BARGE_SHOOT_ROTATION.unaryMinus()));

        private static final Distance LOLLIPOP_X = Inches.of(48);
        public static final Pose2d[] LOLLIPOP_POSES = {
            new Pose2d(LOLLIPOP_X, Inches.of(158.5 - 72), Rotation2d.fromDegrees(225)),
            new Pose2d(LOLLIPOP_X, Inches.of(158.5), Rotation2d.k180deg),
            new Pose2d(LOLLIPOP_X, Inches.of(158.5 + 72), Rotation2d.fromDegrees(135)),
        };
    }

    public static class PathConstants {
        public static final Distance SIDE_DISTANCE = Meters.of(3);

        public static final Distance DISTANCE_TO_REEF =
                Dimensions.ROBOT_SIZE.div(2).plus(Inches.of(2.5));
        public static final Distance DISTANCE_TO_PROCESSOR = Inches.of(29.0 / 2).plus(Dimensions.BUMPER_THICKNESS);

        public static final Distance APPROACH_DISTANCE = Inches.of(30); // *extra* distance to reef when
        // approaching
        public static final Distance PULL_DISTANCE = Inches.of(8);
        public static final Distance STAGE1_DEPLOY_DISTANCE = Inches.of(30);
        public static final Distance STAGE2_DEPLOY_DISTANCE = Inches.of(50);
        public static final Distance STAGE3_DEPLOY_DISTANCE = Meters.of(100); // effectively infinite
        public static final Distance ALGAE_DEPLOY_DISTANCE = Inches.of(15);
        public static final Distance ALGAE_EXTRA_DISTANCE_IN = Inches.of(5);
        public static final Distance FLIP_DISTANCE = Inches.of(80);
        public static final Distance LOLLIPOP_INTAKE_DISTANCE = Inches.of(40);
        public static final Distance CORAL_APPROACH_DISTANCE = Inches.of(20);
        public static final Distance SWITCH_TO_REEFVISION_DISTANCE = Inches.of(30);
        public static final Distance BACKUP_FROM_WALL_DISTANCE = Inches.of(30);

        public static final Distance TRAVERSE_DISTANCE = Inches.of(40); // *extra* distance to reef when moving
        // around to other side

        public static final Time INTAKE_WAIT_TIME = Seconds.of(0.75);
        public static final Time ELEVATOR_SETTLE_TIME = Seconds.of(0.1); // for L1-L3
        public static final Time AFTER_WAIT_TIME = Seconds.of(0.1);
        public static final Time BARGE_SETTLE_TIME = Seconds.of(0.2);

        public static final Distance MIN_PATH_DISTANCE = Inches.of(30);

        public static final LinearVelocity MIN_PATH_SPEED = MetersPerSecond.of(1.5);

        public static final double APPROACH_PROPORTION = 1; // proportion of distance to final waypoint to use approach
        // constraints
        public static final double FAST_PROPORTION = 0.5; // proportion of first waypoint to use fast constraints

        public static final Distance SWEEP_SIDE_DISTANCE = Inches.of(40);
        public static final Distance SWEEP_OFFSET = Inches.of(2);
        public static final double SWEEP_RELATIVE_POS =
                SWEEP_SIDE_DISTANCE.div(Inches.of(12.94 / 2)).magnitude();

        public static final Distance L1_SIDE_DISTANCE = Inches.of(16);
        public static final double L1_RELATIVE_POS =
                L1_SIDE_DISTANCE.div(Inches.of(12.94 / 2)).magnitude();
        public static final Distance BRANCH_INSET = Inches.of(1.6);

        // PathPlanner

        // output: m/s, measure: m
        public static final PIDConstants PP_TRANSLATIONAL_PID = new PIDConstants(3, 0.1, 0.01);
        // output: rad/s, measure: rad
        public static final PIDConstants PP_ROTATIONAL_PID = new PIDConstants(3, 0.1, 0.5);

        public static final PathConstraints FAST_CONSTRAINTS = new PathConstraints(
                MetersPerSecond.of(5.0),
                MetersPerSecondPerSecond.of(4.5),
                RotationsPerSecond.of(1.25),
                RotationsPerSecondPerSecond.of(1.25));

        public static final PathConstraints CONSTRAINTS = new PathConstraints(
                MetersPerSecond.of(3.1),
                MetersPerSecondPerSecond.of(3.0),
                RotationsPerSecond.of(1.25),
                RotationsPerSecondPerSecond.of(1.25));

        public static final PathConstraints APPROACH_CONSTRAINTS = new PathConstraints(
                MetersPerSecond.of(2.0),
                MetersPerSecondPerSecond.of(2.0),
                RotationsPerSecond.of(1.25),
                RotationsPerSecondPerSecond.of(1.25));

        private static final Mass ROBOT_MASS = Pounds.of(130);
        private static final MomentOfInertia ROBOT_MOI = KilogramSquareMeters.of(10.13);
        private static final double WHEEL_COF = 1.2;

        public static final RobotConfig PP_CONFIG = new RobotConfig(
                ROBOT_MASS,
                ROBOT_MOI,
                new ModuleConfig(
                        SwerveConstants.WHEEL_RADIUS,
                        SwerveConstants.SPEED_AT_12V,
                        WHEEL_COF,
                        DCMotor.getKrakenX60Foc(1).withReduction(SwerveConstants.DRIVE_GEAR_RATIO),
                        SwerveConstants.SLIP_CURRENT,
                        1),
                SwerveConstants.GET_MODULE_POSITIONS.get());
    }

    public static class NTConstants {
        public static final LoggedNetworkString AUTO_DESCRIPTOR_ENTRY =
                new LoggedNetworkString("/Tuning/Auto Descriptor", "");
        public static final LoggedNetworkString REEF_TELEOP_AUTO_ENTRY =
                new LoggedNetworkString("/Tuning/Reef Descriptor", "A4");
        public static final LoggedNetworkString ALGAE_SCORE_ENTRY =
                new LoggedNetworkString("/Tuning/Algae Descriptor", "1");
    }

    private Constants() {}
}
