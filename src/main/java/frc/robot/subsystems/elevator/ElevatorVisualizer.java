// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.Constants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.Mode;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

/**
 * Visualizer for elevator and algae manipualtor that includes Mechanism2d and 3d. Also visualizes currently held game
 * pieces
 */
public class ElevatorVisualizer {
    private final String name;
    private final BooleanSupplier hasCoral;
    private final BooleanSupplier algaeDeployed;
    private final BooleanSupplier hasAlgae;
    private final Supplier<Pose2d> poseSupplier;

    private final LoggedMechanism2d mechanism =
            new LoggedMechanism2d(Inches.of(30).in(Meters), Inches.of(95).in(Meters), new Color8Bit(Color.kAliceBlue));
    private final Translation2d rootPos = new Translation2d(0.47, 0.04);
    private final LoggedMechanismLigament2d ligament;
    private final LoggedMechanismLigament2d algaeManipulatorLigamentBottom;

    public ElevatorVisualizer(
            String name,
            BooleanSupplier hasCoral,
            BooleanSupplier algaeDeployed,
            BooleanSupplier hasAlgae,
            Supplier<Pose2d> poseSupplier) {
        this.name = name;
        this.hasCoral = hasCoral;
        this.algaeDeployed = algaeDeployed;
        this.hasAlgae = hasAlgae;
        this.poseSupplier = poseSupplier;

        LoggedMechanismRoot2d root = mechanism.getRoot(name + " Root", 0.6, 0.1);
        ligament = root.append(new LoggedMechanismLigament2d(
                name + "Elevator", Inches.of(26).in(Meters), 90, 4.0, new Color8Bit(Color.kBlueViolet)));

        ligament.append(new LoggedMechanismLigament2d(
                name + "AlgaeManipulatorTop", Inches.of(14).in(Meters), -20, 2.0, new Color8Bit(Color.kGreen)));
        algaeManipulatorLigamentBottom = ligament.append(new LoggedMechanismLigament2d(
                name + "AlgaeManipulatorBottom", Inches.of(6).in(Meters), -25, 2.0, new Color8Bit(Color.kGreen)));
    }

    public void update(Distance height) {
        if (Constants.CURRENT_MODE == Mode.REAL) return;

        ligament.setLength(height.in(Meters));
        algaeManipulatorLigamentBottom.setAngle(algaeDeployed.getAsBoolean() ? -90 : -25);
        Logger.recordOutput("Mechanism2d/" + name, mechanism);

        Pose3d[] coral;
        if (hasCoral.getAsBoolean()) {
            coral = new Pose3d[] {
                new Pose3d(poseSupplier.get())
                        .transformBy(new Transform3d(
                                new Translation3d(rootPos.getX() / 2, 0, rootPos.getY() + height.in(Meters)),
                                new Rotation3d(0, ElevatorConstants.SHOOT_ANGLE.in(Radians), 0)))
            };
        } else {
            coral = new Pose3d[] {};
        }

        Pose3d[] algae;
        if (hasAlgae.getAsBoolean()) {
            algae = new Pose3d[] {
                new Pose3d(poseSupplier.get())
                        .transformBy(new Transform3d(
                                new Translation3d(
                                        rootPos.getX() / 2 + 0.15, 0, rootPos.getY() + height.in(Meters) + 0.2),
                                new Rotation3d(0, 0, 0)))
            };
        } else {
            algae = new Pose3d[] {};
        }

        Logger.recordOutput("Mechanism3d/" + name + "/Coral", coral);
        Logger.recordOutput("Mechanism3d/" + name + "/Algae", algae);

        double stage3Height = MathUtil.clamp(height.in(Meters), 0, ElevatorConstants.MAX_HEIGHT.in(Meters));
        double stage2Height = MathUtil.clamp(
                height.in(Meters) - ElevatorConstants.STAGE2_HEIGHT.in(Meters),
                0,
                ElevatorConstants.STAGE1_HEIGHT
                        .plus(ElevatorConstants.STAGE2_HEIGHT)
                        .in(Meters));
        double stage1Height = MathUtil.clamp(
                height.in(Meters) - ElevatorConstants.STAGE1_HEIGHT.in(Meters),
                0,
                ElevatorConstants.STAGE2_HEIGHT.in(Meters));

        Logger.recordOutput("Mechanism3d/Zero", new Pose3d());
        Logger.recordOutput(
                "Mechanism3d/" + name + "/Stage1",
                new Pose3d(new Translation3d(0.09, 0, stage1Height + 0.143), new Rotation3d(0, 0, 0)));
        Logger.recordOutput(
                "Mechanism3d/" + name + "/Stage2",
                new Pose3d(new Translation3d(0.09, 0, stage2Height + 0.169), new Rotation3d(0, 0, 0)));
        Logger.recordOutput(
                "Mechanism3d/" + name + "/EndEffector",
                new Pose3d(
                        new Translation3d(0.09, 0, stage3Height - ElevatorConstants.MIN_HEIGHT.in(Meters) + 0.195),
                        new Rotation3d(0, 0, 0)));
        Logger.recordOutput(
                "Mechanism3d/" + name + "/Hinge",
                new Pose3d(
                        new Translation3d(0.28, 0, stage3Height - ElevatorConstants.MIN_HEIGHT.in(Meters) + 0.36),
                        new Rotation3d(
                                0,
                                algaeDeployed.getAsBoolean()
                                        ? 0
                                        : Degrees.of(-80).in(Radians),
                                0)));
    }
}
