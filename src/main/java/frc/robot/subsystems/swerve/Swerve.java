// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.swerve;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.util.PathPlannerLogging;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.MutLinearVelocity;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Constants;
import frc.robot.Constants.Mode;
import frc.robot.Constants.PathConstants;
import frc.robot.Constants.SwerveConstants;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import java.util.function.Supplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Swerve extends SubsystemBase {
    static final Lock odometryLock = new ReentrantLock();

    private final GyroIO gyroIO;
    private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();
    private final Module[] modules = new Module[4];
    private final SysIdRoutine sysIdRoutine;
    private final Alert gyroDisconnectedAlert =
            new Alert("Disconnected gyro, using kinematics as fallback.", Alert.AlertType.kError);
    private final Field2d field2d = new Field2d();

    private SwerveDriveKinematics kinematics = new SwerveDriveKinematics(SwerveConstants.GET_MODULE_POSITIONS.get());
    private Rotation2d rawGyroRotation = new Rotation2d();
    private SwerveModulePosition[] lastModulePositions = // For delta tracking
            new SwerveModulePosition[] {
                new SwerveModulePosition(),
                new SwerveModulePosition(),
                new SwerveModulePosition(),
                new SwerveModulePosition()
            };
    private SwerveDrivePoseEstimator poseEstimator =
            new SwerveDrivePoseEstimator(kinematics, rawGyroRotation, lastModulePositions, new Pose2d());
    private Supplier<Double> externalSpeedScale = () -> 1.0;

    public Swerve(GyroIO gyroIO, ModuleIO flModuleIO, ModuleIO frModuleIO, ModuleIO blModuleIO, ModuleIO brModuleIO) {
        this.gyroIO = gyroIO;
        modules[0] = new Module(flModuleIO, 0, SwerveConstants.FrontLeft.MODULE_CONSTANTS);
        modules[1] = new Module(frModuleIO, 1, SwerveConstants.FrontRight.MODULE_CONSTANTS);
        modules[2] = new Module(blModuleIO, 2, SwerveConstants.BackLeft.MODULE_CONSTANTS);
        modules[3] = new Module(brModuleIO, 3, SwerveConstants.BackRight.MODULE_CONSTANTS);

        // Start odometry thread
        PhoenixOdometryThread.getInstance().start();

        // Configure PP AutoBuilder
        AutoBuilder.configure(
                this::getPose,
                this::setPose,
                this::getChassisSpeeds,
                this::drive,
                new PPHolonomicDriveController(PathConstants.PP_TRANSLATIONAL_PID, PathConstants.PP_ROTATIONAL_PID),
                PathConstants.PP_CONFIG,
                () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red,
                this);
        PathPlannerLogging.setLogActivePathCallback((activePath) -> {
            Logger.recordOutput("Odometry/Trajectory", activePath.toArray(new Pose2d[activePath.size()]));
        });
        PathPlannerLogging.setLogTargetPoseCallback((targetPose) -> {
            Logger.recordOutput("Odometry/TrajectorySetpoint", targetPose);
        });

        // Configure SysId
        sysIdRoutine = new SysIdRoutine(
                new SysIdRoutine.Config(
                        null, // Use default ramp rate (1 V/s)
                        Volts.of(4), // Reduce dynamic step voltage to 4 V to prevent
                        // brownout
                        null, // Use default timeout (10 s)
                        // Log state with SignalLogger class
                        (state) -> Logger.recordOutput("Swerve/SysIdTranslation_State", state.toString())),
                new SysIdRoutine.Mechanism((output) -> runCharacterization(output), null, this));

        SmartDashboard.putData("SysId Dynamic Forward", sysIdDynamic(Direction.kForward));
        SmartDashboard.putData("SysId Dynamic Reverse", sysIdDynamic(Direction.kReverse));
        SmartDashboard.putData("SysId Quasi Forward", sysIdQuasistatic(Direction.kForward));
        SmartDashboard.putData("SysId Quasi Reverse", sysIdQuasistatic(Direction.kReverse));
        SmartDashboard.putData(field2d);
    }

    @Override
    public void periodic() {
        odometryLock.lock(); // Prevents odometry updates while reading data

        gyroIO.updateInputs(gyroInputs);
        Logger.processInputs("Swerve/Gyro", gyroInputs);

        for (var module : modules) {
            module.periodic();
        }

        odometryLock.unlock();

        // Log empty setpoint states when disabled
        if (DriverStation.isDisabled()) {
            Logger.recordOutput("SwerveStates/Setpoints", new SwerveModuleState[] {});
            Logger.recordOutput("SwerveStates/SetpointsOptimized", new SwerveModuleState[] {});
        }

        updateOdometry();

        // Update gyro alert
        gyroDisconnectedAlert.set(!gyroInputs.connected && Constants.CURRENT_MODE != Mode.SIM);
    }

    private void updateOdometry() {
        field2d.setRobotPose(getPose());
        double[] sampleTimestamps = modules[0].getOdometryTimestamps();
        int sampleCount = sampleTimestamps.length;

        for (int i = 0; i < sampleCount; i++) {
            SwerveModulePosition[] modulePositions = new SwerveModulePosition[4];
            SwerveModulePosition[] moduleDeltas = new SwerveModulePosition[4];
            for (int j = 0; j < 4; j++) {
                modulePositions[j] = modules[j].getOdometryPositions()[i];
                moduleDeltas[j] = new SwerveModulePosition(
                        modulePositions[j].distanceMeters - lastModulePositions[j].distanceMeters,
                        modulePositions[j].angle);

                lastModulePositions[j] = modulePositions[j];
            }
            // Update gyro
            if (gyroInputs.connected) {
                rawGyroRotation = gyroInputs.odometryYawPositions[i];
            } else {
                // Use angle delta from kinematics
                Twist2d twist = kinematics.toTwist2d(moduleDeltas);
                rawGyroRotation = rawGyroRotation.plus(new Rotation2d(twist.dtheta));
            }

            // Apply update
            poseEstimator.updateWithTime(sampleTimestamps[i], rawGyroRotation, modulePositions);
        }
    }

    public void drive(ChassisSpeeds speeds) {
        // Calculate module setpoints
        ChassisSpeeds discreteSpeeds = ChassisSpeeds.discretize(speeds, 0.02);
        double scale = Math.max(0.0, externalSpeedScale.get());
        discreteSpeeds.vxMetersPerSecond *= scale;
        discreteSpeeds.vyMetersPerSecond *= scale;
        discreteSpeeds.omegaRadiansPerSecond *= scale;
        SwerveModuleState[] setpointStates = kinematics.toSwerveModuleStates(discreteSpeeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(setpointStates, SwerveConstants.SPEED_AT_12V);

        // Log unoptimized setpoints and setpoint speeds
        Logger.recordOutput("SwerveStates/Setpoints", setpointStates);
        Logger.recordOutput("SwerveChassisSpeeds/Setpoints", discreteSpeeds);

        // Send setpoints to modules
        for (int i = 0; i < 4; i++) {
            modules[i].applySetpoint(setpointStates[i]);
        }

        // Log optimized setpoints (applySetpoint mutates each state)
        Logger.recordOutput("SwerveStates/SetpointsOptimized", setpointStates);
    }

    public void driveFieldCentric(LinearVelocity xVel, LinearVelocity yVel, AngularVelocity omega) {
        drive(ChassisSpeeds.fromFieldRelativeSpeeds(xVel, yVel, omega, getRotation()));
    }

    public void setExternalSpeedScale(Supplier<Double> scaleSupplier) {
        externalSpeedScale = scaleSupplier != null ? scaleSupplier : () -> 1.0;
    }

    /** Runs the drive in a straight line with the specified drive output. */
    public void runCharacterization(Voltage output) {
        for (int i = 0; i < 4; i++) {
            modules[i].runCharacterization(output);
        }
    }

    /** Stop moving, but don't put wheels in an X pattern */
    public void stop() {
        drive(new ChassisSpeeds());
    }

    /** Stop moving and put wheels in an X pattern */
    public void brake() {
        Rotation2d[] headings = new Rotation2d[4];
        for (int i = 0; i < 4; i++) {
            headings[i] = SwerveConstants.GET_MODULE_POSITIONS.get()[i].getAngle();
        }
        kinematics.resetHeadings(headings);
        stop();
    }

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return run(() -> runCharacterization(Volts.zero()))
                .withTimeout(1.0)
                .andThen(sysIdRoutine.quasistatic(direction));
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return run(() -> runCharacterization(Volts.zero())).withTimeout(1.0).andThen(sysIdRoutine.dynamic(direction));
    }

    /** Returns the module states (turn angles and drive velocities) for all of the modules. */
    @AutoLogOutput(key = "SwerveStates/Measured")
    private SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[4];
        for (int i = 0; i < 4; i++) {
            states[i] = modules[i].getState();
        }
        return states;
    }

    /** Returns the module positions (turn angles and drive positions) for all of the modules. */
    private SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] states = new SwerveModulePosition[4];
        for (int i = 0; i < 4; i++) {
            states[i] = modules[i].getPosition();
        }
        return states;
    }

    /** Returns the measured chassis speeds of the robot. */
    @AutoLogOutput(key = "SwerveChassisSpeeds/Measured")
    public ChassisSpeeds getChassisSpeeds() {
        return kinematics.toChassisSpeeds(getModuleStates());
    }

    public ChassisSpeeds getFieldSpeeds() {
        return ChassisSpeeds.fromRobotRelativeSpeeds(getChassisSpeeds(), getRotation());
    }

    /** Returns the average velocity of the modules in rotations/sec (Phoenix native units). */
    public LinearVelocity getFFCharacterizationVelocity() {
        MutLinearVelocity output = MetersPerSecond.zero().mutableCopy();
        for (int i = 0; i < 4; i++) {
            output.mut_plus(modules[i].getDriveVelocity().div(4));
        }
        return output;
    }

    /** Returns the current odometry pose. */
    @AutoLogOutput(key = "Odometry/Robot")
    public Pose2d getPose() {
        return poseEstimator.getEstimatedPosition();
    }

    /** Returns the current odometry rotation. */
    public Rotation2d getRotation() {
        return getPose().getRotation();
    }

    /** Resets the current odometry pose. */
    public void setPose(Pose2d pose) {
        poseEstimator.resetPosition(rawGyroRotation, getModulePositions(), pose);
    }

    /** Adds a new timestamped vision measurement. */
    public void addVisionMeasurement(
            Pose2d visionRobotPoseMeters, Time timestamp, Matrix<N3, N1> visionMeasurementStdDevs) {
        poseEstimator.addVisionMeasurement(visionRobotPoseMeters, timestamp.in(Seconds), visionMeasurementStdDevs);
    }
}
