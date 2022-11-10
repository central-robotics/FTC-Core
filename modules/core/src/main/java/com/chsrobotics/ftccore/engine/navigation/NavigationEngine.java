package com.chsrobotics.ftccore.engine.navigation;

import com.chsrobotics.ftccore.engine.localization.LocalizationEngine;
import com.chsrobotics.ftccore.engine.navigation.path.ParametricSpline;
import com.chsrobotics.ftccore.geometry.Position;
import com.chsrobotics.ftccore.hardware.HardwareManager;
import com.chsrobotics.ftccore.engine.navigation.control.*;
import com.chsrobotics.ftccore.utilities.MathUtil;

import java.util.List;

public class NavigationEngine {
    private final HardwareManager hardware;
    public final LocalizationEngine localization;
    private final PID linearController;
    private final PID rotationController;
    private final SplineHelper splineHelper;
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
        this.linearController = hardware.linearCtrler;
        this.rotationController = hardware.rotCtrler;

        splineHelper = new SplineHelper();
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

        if (Math.abs(destination.t - (position.t + (2 * Math.PI))) < thetaError)
        {
            thetaError = destination.t - (position.t + (2 * Math.PI));
            isCounterClockwise = true;
        }

        if (thetaError > 0 && (thetaError < Math.PI))
            isCounterClockwise = true;

        if (thetaError < 0 && (thetaError > -Math.PI))
            isCounterClockwise = false;

//        hardware.opMode.telemetry.addData("Ep", error);
//        hardware.opMode.telemetry.addData("Et", thetaError);
//        hardware.opMode.telemetry.addData("Ct", position.t);
//        hardware.opMode.telemetry.addData("Dt", destination.t);
//        hardware.opMode.telemetry.addData("V", MathUtil.calculateDistance(position, localization.lastPosition));
//        if (error >= 10) {
//            hardware.opMode.telemetry.addLine("Position");
//        }
//        if (thetaError >= 0.02) {
//            hardware.opMode.telemetry.addLine("Theta");
//        }
        return (error < 20 && Math.abs(thetaError) < 0.2); //&& MathUtil.calculateDistance(position, localization.lastPosition) < 5);
    }

    public void navigateInALinearFashion(Position destination)
    {
        while (hardware.debugMode) {
            position = localization.getCurrentPosition();
            hardware.opMode.telemetry.addData("X", position.x);
            hardware.opMode.telemetry.addData("Y", position.y);
            hardware.opMode.telemetry.addData("T", position.t);
            hardware.opMode.telemetry.update();
        }

        while (!isTargetReached(destination) && !hardware.opMode.isStopRequested())
        {
            position = localization.getCurrentPosition();

            double orientation, negOutput, posOutput;

            if (destination.x - position.x > 0)
                orientation = Math.atan(linearController.getSlope(destination, position)) - Math.PI / 4 - position.t;
            else if (destination.x - position.x < 0)
                orientation = Math.atan(linearController.getSlope(destination, position)) + Math.PI - Math.PI / 4 - position.t;
            else
                orientation = Math.PI / 2;

            magnitude = linearController.getOutput(error, 0);

            negOutput = magnitude * Math.sin(orientation);

            if (orientation == 0) {
                posOutput = negOutput;
            } else {
                posOutput = magnitude * Math.cos(orientation);
            }

            double thetaOutput = rotationController.getOutput(Math.abs(thetaError), 0);


            hardware.getLeftFrontMotor().setVelocity((-posOutput) - ((isCounterClockwise ? -1 : 1) * thetaOutput));
            hardware.getRightFrontMotor().setVelocity(( negOutput) - ((isCounterClockwise ? -1 : 1) * thetaOutput));
            hardware.getLeftBackMotor().setVelocity((-negOutput) - ((isCounterClockwise ? -1 : 1) * thetaOutput));
            hardware.getRightBackMotor().setVelocity((posOutput) - ((isCounterClockwise ? -1 : 1) * thetaOutput));
        }
        linearController.resetSum();
        rotationController.resetSum();
    }

    public void navigateInANonLinearFashion(List<Position> positions)
    {
        double distTraveled = 0;
        Position lastPosition = localization.getCurrentPosition();
        ParametricSpline spline = splineHelper.computeSpline(List<Position> positions);

        while (!isTargetReached(positions.get(positions.size() - 1)))
        {
            position = localization.getCurrentPosition();

            double t, orientation, negOutput, posOutput;

            distTraveled += Math.sqrt(Math.pow(position.x - lastPosition.x, 2) + Math.pow(position.y - lastPosition.y, 2));


            magnitude = linearController.getOutput(error, 0);

            negOutput = magnitude * Math.sin(orientation);

            if (orientation == 0) {
                posOutput = negOutput;
            } else {
                posOutput = magnitude * Math.cos(orientation);
            }

            double thetaOutput = rotationController.getOutput(Math.abs(thetaError), 0);

            lastPosition = position;

            hardware.getLeftFrontMotor().setVelocity((-posOutput) - ((isCounterClockwise ? -1 : 1) * thetaOutput));
            hardware.getRightFrontMotor().setVelocity(( negOutput) - ((isCounterClockwise ? -1 : 1) * thetaOutput));
            hardware.getLeftBackMotor().setVelocity((-negOutput) - ((isCounterClockwise ? -1 : 1) * thetaOutput));
            hardware.getRightBackMotor().setVelocity((posOutput) - ((isCounterClockwise ? -1 : 1) * thetaOutput));
        }
    }
}
