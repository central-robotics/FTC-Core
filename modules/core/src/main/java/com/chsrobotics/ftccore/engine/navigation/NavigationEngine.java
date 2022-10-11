package com.chsrobotics.ftccore.engine.navigation;

import com.chsrobotics.ftccore.engine.localization.LocalizationEngine;
import com.chsrobotics.ftccore.geometry.Position;
import com.chsrobotics.ftccore.hardware.HardwareManager;

public class NavigationEngine {
    private final HardwareManager hardware;
    public final LocalizationEngine localization;
    private final PID controller;
    private final PID thetaController;
    private final SplineController splineController;
    public Position position = new Position();
    double t;
    double magnitude;

    public NavigationEngine(HardwareManager hardware)
    {
        this.hardware = hardware;
    }

    public void navigateInALinearFashion(Position destination)
    {
        position = localization.getCurrentPosition();

        double orientation, negOutput, posOutput;

        if (destination.x - position.x > 0)
            orientation = Math.atan(controller.getSlope(destination, position)) - Math.PI / 4 - position.t;
        else if (destination.x - position.x < 0)
            orientation = Math.atan(controller.getSlope(destination, position)) + Math.PI - Math.PI / 4 - position.t;
        else
            orientation = Math.PI / 2;

        double error = Math.sqrt(Math.pow(destination.y - position.y, 2) + Math.pow(destination.x - position.x, 2));
        double thetaError = destination.t - position.t;

        magnitude = controller.getOutput(error);

        negOutput = magnitude * Math.sin(orientation);

        if (orientation == 0) {
            posOutput = negOutput;
        } else {
            posOutput = magnitude * Math.cos(orientation);
        }

        double thetaOutput = thetaController.getOutput(Math.abs(thetaError));

        hardware.getLeftFrontMotor().setPower((-0.1 * posOutput) - thetaOutput);
        hardware.getRightFrontMotor().setPower((0.1 * negOutput) - thetaOutput);
        hardware.getLeftBackMotor().setPower((-0.1 * negOutput) - thetaOutput);
        hardware.getRightBackMotor().setPower((0.1 * posOutput) - thetaOutput);
    }
}
