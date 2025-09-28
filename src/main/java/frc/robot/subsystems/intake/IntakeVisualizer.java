// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.Constants;
import frc.robot.Constants.Mode;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

/** Add your docs here. */
public class IntakeVisualizer {
    private final BooleanSupplier coralDetected;
    private final Supplier<Pose2d> poseSupplier;

    private final LoggedMechanism2d mechanism =
            new LoggedMechanism2d(Inches.of(40).in(Meters), Inches.of(40).in(Meters), new Color8Bit(Color.kAliceBlue));
    private final LoggedMechanismLigament2d armLigament;
    private final LoggedMechanismLigament2d rollersLigament;

    public IntakeVisualizer(BooleanSupplier coralDected, Supplier<Pose2d> poseSupplier) {
        this.coralDetected = coralDected;
        this.poseSupplier = poseSupplier;

        LoggedMechanismRoot2d root = mechanism.getRoot("IntakeRoot", 0.4, 0.1);
        armLigament = root.append(new LoggedMechanismLigament2d(
                "IntakeArm", Inches.of(15).in(Meters), 90, 4.0, new Color8Bit(Color.kBlueViolet)));
        rollersLigament = armLigament.append(new LoggedMechanismLigament2d(
                "IntakeRollers", Inches.of(8).in(Meters), 60, 4.0, new Color8Bit(Color.kBlueViolet)));
    }

    public void update(Angle position) {
        if (Constants.CURRENT_MODE == Mode.REAL) return;

        armLigament.setAngle(160 - position.in(Degrees));
        Logger.recordOutput("Mechanism2d/" + "Intake", mechanism);

        Pose3d[] coral;
        if (coralDetected.getAsBoolean()) {
            rollersLigament.setAngle(40);
            coral = new Pose3d[] {
                new Pose3d(poseSupplier.get())
                        .transformBy(new Transform3d(
                                new Translation3d(-Inches.of(20).in(Meters), 0, 0),
                                new Rotation3d(Degrees.zero(), Degrees.zero(), Degrees.of(90))))
            };
        } else {
            rollersLigament.setAngle(60);
            coral = new Pose3d[] {};
        }

        Logger.recordOutput("Mechanism3d/Intake/Coral", coral);
        Logger.recordOutput(
                "Mechanism3d/Intake/Intake",
                new Pose3d(new Translation3d(-0.12, 0, 0.2), new Rotation3d(0, position.in(Radians), 0)));
    }
}
