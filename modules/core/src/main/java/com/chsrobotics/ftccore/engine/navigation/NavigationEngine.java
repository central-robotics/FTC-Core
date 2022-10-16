package com.chsrobotics.ftccore.engine.navigation;

import com.chsrobotics.ftccore.engine.localization.LocalizationEngine;
import com.chsrobotics.ftccore.geometry.Position;
import com.chsrobotics.ftccore.hardware.HardwareManager;
import com.chsrobotics.ftccore.engine.navigation.control.*;

public class NavigationEngine {
    private final HardwareManager hardware;
    public final LocalizationEngine localization;
    private final PID linearCtrler;
    private final PID rotCtrler;
    private final SplineController splineController;
    public Position position = new Position();
    double t;
    double magnitude;

    public NavigationEngine(LocalizationEngine localization, HardwareManager hardware)
    {
        this.hardware = hardware;
        this.localization = localization;
        this.linearCtrler = hardware.linearCtrler;
        this.rotCtrler = hardware.rotCtrler;

        splineController = new SplineController();
    }

    public void navigateInALinearFashion(Position destination)
    {
        position = localization.getCurrentPosition();

        double orientation, negOutput, posOutput;

        if (destination.x - position.x > 0)
            orientation = Math.atan(linearCtrler.getSlope(destination, position)) - Math.PI / 4 - position.t;
        else if (destination.x - position.x < 0)
            orientation = Math.atan(linearCtrler.getSlope(destination, position)) + Math.PI - Math.PI / 4 - position.t;
        else
            orientation = Math.PI / 2;

        double error = Math.sqrt(Math.pow(destination.y - position.y, 2) + Math.pow(destination.x - position.x, 2));
        double thetaError = destination.t - position.t;

        magnitude = linearCtrler.getOutput(error, 0);

        negOutput = magnitude * Math.sin(orientation);

        if (orientation == 0) {
            posOutput = negOutput;
        } else {
            posOutput = magnitude * Math.cos(orientation);
        }

        double thetaOutput = rotCtrler.getOutput(Math.abs(thetaError), 0);

        hardware.getLeftFrontMotor().setPower((-0.1 * posOutput) - thetaOutput);
        hardware.getRightFrontMotor().setPower((0.1 * negOutput) - thetaOutput);
        hardware.getLeftBackMotor().setPower((-0.1 * negOutput) - thetaOutput);
        hardware.getRightBackMotor().setPower((0.1 * posOutput) - thetaOutput);
    }
}
