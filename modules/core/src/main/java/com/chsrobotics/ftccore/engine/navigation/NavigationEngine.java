package com.chsrobotics.ftccore.engine.navigation;

import com.chsrobotics.ftccore.engine.localization.LocalizationEngine;
import com.chsrobotics.ftccore.geometry.Position;
import com.chsrobotics.ftccore.hardware.HardwareManager;
import com.chsrobotics.ftccore.engine.navigation.control.*;
import com.chsrobotics.ftccore.utilities.MathUtil;

public class NavigationEngine {
    private final HardwareManager hardware;
    public final LocalizationEngine localization;
    private final PID linearCtrler;
    private final PID rotCtrler;
    private final SplineController splineController;
    public Position position = new Position();
    double t;
    double magnitude;
    private double error;
    private double thetaError;
    private boolean isCounterClockwise;

    public NavigationEngine(LocalizationEngine localization, HardwareManager hardware)
    {
        this.hardware = hardware;
        this.localization = localization;
        this.linearCtrler = hardware.linearCtrler;
        this.rotCtrler = hardware.rotCtrler;

        splineController = new SplineController();
    }

    private boolean isTargetReached(Position destination)
    {
        error = Math.sqrt(Math.pow(destination.y - position.y, 2) + Math.pow(destination.x - position.x, 2));
        thetaError = destination.t - position.t;
        isCounterClockwise = false;

        if (Math.abs(destination.t - (position.t - (2 * Math.PI))) < Math.abs(thetaError))
        {
            thetaError = destination.t - (position.t - (2 * Math.PI));
            isCounterClockwise = true;
        }

        if (Math.abs(destination.t - (position.t + (2 * Math.PI))) < Math.abs(thetaError))
        {
            thetaError = destination.t - (position.t + (2 * Math.PI));
            isCounterClockwise = true;
        }

        if (thetaError > 0 && (thetaError < Math.PI))
            isCounterClockwise = true;

        if (thetaError < 0 && (thetaError > -Math.PI))
            isCounterClockwise = false;

        hardware.opMode.telemetry.addData("Et", thetaError);
        hardware.opMode.telemetry.addData("Ct", position.t);
        hardware.opMode.telemetry.addData("Dt", destination.t);
        hardware.opMode.telemetry.addData("V", MathUtil.calculateDistance(position, localization.lastPosition));
        hardware.opMode.telemetry.update();
        return (error < 10 && Math.abs(thetaError) < 0.05 && MathUtil.calculateDistance(position, localization.lastPosition) < 5);
    }

    public void navigateInALinearFashion(Position destination)
    {
        while (!isTargetReached(destination) &&  !hardware.opMode.isStopRequested())
        {
            position = localization.getCurrentPosition();

            double orientation, negOutput, posOutput;

            if (destination.x - position.x > 0)
                orientation = Math.atan(linearCtrler.getSlope(destination, position)) - Math.PI / 4 - position.t;
            else if (destination.x - position.x < 0)
                orientation = Math.atan(linearCtrler.getSlope(destination, position)) + Math.PI - Math.PI / 4 - position.t;
            else
                orientation = Math.PI / 2;

            magnitude = linearCtrler.getOutput(error, 0);

            negOutput = magnitude * Math.sin(orientation);

            if (orientation == 0) {
                posOutput = negOutput;
            } else {
                posOutput = magnitude * Math.cos(orientation);
            }

            double thetaOutput = rotCtrler.getOutput(Math.abs(thetaError), 0);

            hardware.getLeftFrontMotor().setPower((-0.1 * posOutput) - (isCounterClockwise ? -1 : 1) * thetaOutput);
            hardware.getRightFrontMotor().setPower((0.1 * negOutput) - (isCounterClockwise ? -1 : 1) * thetaOutput);
            hardware.getLeftBackMotor().setPower((-0.1 * negOutput) - (isCounterClockwise ? -1 : 1) * thetaOutput);
            hardware.getRightBackMotor().setPower((0.1 * posOutput) - (isCounterClockwise ? -1 : 1) * thetaOutput);
        }
    }
}
