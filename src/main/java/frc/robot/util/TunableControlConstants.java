// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

public class TunableControlConstants {
    // PID gains
    LoggedTunableNumber kP;
    LoggedTunableNumber kI;
    LoggedTunableNumber kD;
    LoggedTunableNumber tolerance;
    LoggedTunableNumber velTolerance;
    LoggedTunableNumber iZone;
    LoggedTunableNumber iMin;
    LoggedTunableNumber iMax;
    double period;

    // feedforward gains
    LoggedTunableNumber kV;
    LoggedTunableNumber kA;

    // physical gains
    LoggedTunableNumber kS;
    LoggedTunableNumber kG;

    // trapezoid profile
    LoggedTunableNumber maxVel;
    LoggedTunableNumber maxAcc;

    // continuous control
    boolean isContinuous = false;
    double maxInput;
    double minInput;

    public TunableControlConstants(String key, ControlConstants constants) {
        this.kP = new LoggedTunableNumber(key+"/kP", constants.kP);
        this.kI = new LoggedTunableNumber(key+"/kI", constants.kI);
        this.kD = new LoggedTunableNumber(key+"/kD", constants.kD);
        this.tolerance = new LoggedTunableNumber(key+"/tolerance", constants.tolerance);
        this.velTolerance = new LoggedTunableNumber(key+"/velTolerance", constants.velTolerance);
        this.iZone = new LoggedTunableNumber(key+"/iZone", constants.iZone);
        this.iMax = new LoggedTunableNumber(key+"/maxIntegral", constants.iMax);
        this.iMin = new LoggedTunableNumber(key+"/minIntegral", constants.iMin);
        this.period = constants.period;
        this.kV = new LoggedTunableNumber(key+"/kV", constants.kV);
        this.kA = new LoggedTunableNumber(key+"/kA", constants.kA);
        this.kS = new LoggedTunableNumber(key+"/kS", constants.kS);
        this.kG = new LoggedTunableNumber(key+"/kG", constants.kG);
        this.maxVel = new LoggedTunableNumber(key+"/maxVel", constants.maxVel);
        this.maxAcc = new LoggedTunableNumber(key+"/maxAcc", constants.maxAcc);
        this.isContinuous = constants.isContinuous;
        this.maxInput = constants.maxInput;
        this.minInput = constants.minInput;
    }

    public PIDController getPIDController() {
        PIDController controller = new PIDController(kP, kI, kD);
        controller.setTolerance(tolerance);
        controller.setIntegratorRange(iMin, iMax);
        controller.setIZone(iZone);

        return controller;
    }

    public ProfiledPIDController getProfiledPIDController() {
        ProfiledPIDController controller =
                new ProfiledPIDController(kP, kI, kD, new TrapezoidProfile.Constraints(maxVel, maxAcc));
        controller.setTolerance(tolerance);
        controller.setIntegratorRange(iMin, iMax);
        controller.setIZone(iZone);

        return controller;
    }

    public ElevatorFeedforward getElevatorFeedforward() {
        return new ElevatorFeedforward(kS, kG, kV, kA);
    }

    public SimpleMotorFeedforward getSimpleFeedforward() {
        return new SimpleMotorFeedforward(kS, kV, kA);
    }
}
