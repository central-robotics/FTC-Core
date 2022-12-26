package com.chsrobotics.ftccore.engine.navigation;

import com.chsrobotics.ftccore.engine.localization.LocalizationEngine;
import com.chsrobotics.ftccore.geometry.Position;
import com.chsrobotics.ftccore.hardware.HardwareManager;
import com.chsrobotics.ftccore.engine.navigation.control.*;

import java.util.List;

public class NavigationEngine {
    private final HardwareManager hardware;
    public final LocalizationEngine localization;
    private final PID linearController;
    private final PID rotationController;
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
    }

    public boolean isTargetReached(Position destination)
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

        return (error < hardware.linearTolerance && Math.abs(thetaError) < hardware.rotTolerance);
    }

    public void navigateInALinearFashion(Position destination)
    {

        position = localization.getCurrentPosition();

        double orientation = Math.atan2(destination.y - position.y, destination.x - position.x) - Math.PI / 4 - position.t;

        magnitude = linearController.getOutput(error, 0);

        double negOutput = magnitude * Math.sin(orientation);

        double posOutput = magnitude * Math.cos(orientation);
        if (orientation == 0) {
            posOutput = negOutput;
        }

        double thetaOutput = Math.abs(thetaError) >= 0.05 ? rotationController.getOutput(Math.abs(thetaError), 0) : 0;

        if (hardware.debugMode) {
            hardware.opMode.telemetry.addData("X", position.x);
            hardware.opMode.telemetry.addData("Y", position.y);
            hardware.opMode.telemetry.addData("T", position.t);
            hardware.opMode.telemetry.addData("error", error);
			hardware.opMode.telemetry.addData("direction", orientation);
            hardware.opMode.telemetry.update();
        }

        hardware.getLeftFrontMotor().setVelocity((-posOutput) + ((isCounterClockwise ? 1 : -1) * thetaOutput));
        hardware.getRightFrontMotor().setVelocity((negOutput) + ((isCounterClockwise ? 1 : -1) * thetaOutput));
        hardware.getLeftBackMotor().setVelocity((-negOutput) + ((isCounterClockwise ? 1 : -1) * thetaOutput));
        hardware.getRightBackMotor().setVelocity((posOutput) + ((isCounterClockwise ? 1 : -1) * thetaOutput));
//
//        hardware.driveMotors[0].setPower(0);
//        hardware.driveMotors[1].setPower(0);
//        hardware.driveMotors[2].setPower(0);
//        hardware.driveMotors[3].setPower(0);
    }

    public void navigateInANonLinearFashion(List<Position> positions)
    {



        double distTraveled = 0;
        Position lastPosition = localization.getCurrentPosition();

        double[] x = new double[positions.size()];
        double[] y = new double[positions.size()];

        for (int i = 0; i < positions.size(); i++) {
            x[i] = positions.get(i).x;
            y[i] = positions.get(i).y;
        }

        SplineHelper splineHelper = new SplineHelper();

        ParametricSpline spline = splineHelper.computeSpline(x, y);

        while (!isTargetReached(positions.get(positions.size() - 1)) && t < 1 && !hardware.opMode.isStopRequested())
        {
            position = localization.getCurrentPosition();

            double orientation, negOutput, posOutput;

            distTraveled += Math.sqrt(Math.pow(position.x - lastPosition.x, 2) + Math.pow(position.y - lastPosition.y, 2)) / 1.2;

            t = distTraveled / spline.splineDistance;

            if (t >= 1)
                break;

            if (spline.xSpline.derivative().value(t) > 0)
                orientation = Math.atan(spline.getDerivative(t)) - Math.PI / 4 - position.t;
            else
                orientation = Math.atan(spline.getDerivative(t)) + Math.PI - Math.PI / 4 - position.t;
            negOutput = 1000 * Math.sin(orientation);

            if (orientation == 0) {
                posOutput = negOutput;
            } else {
                posOutput = 1000 * Math.cos(orientation);
            }

            double thetaOutput = rotationController.getOutput(Math.abs(thetaError), 0);

            lastPosition = position;
            hardware.opMode.telemetry.addData("t", t);
            hardware.opMode.telemetry.addData("magnitude", magnitude);
            hardware.opMode.telemetry.addData("dist traveled", distTraveled);
            hardware.opMode.telemetry.addData("pos", posOutput);
            hardware.opMode.telemetry.addData("neg", negOutput);
            hardware.opMode.telemetry.update();

            hardware.getLeftFrontMotor().setVelocity((-posOutput) - ((isCounterClockwise ? -1 : 1) * thetaOutput));
            hardware.getRightFrontMotor().setVelocity(( negOutput) - ((isCounterClockwise ? -1 : 1) * thetaOutput));
            hardware.getLeftBackMotor().setVelocity((-negOutput) - ((isCounterClockwise ? -1 : 1) * thetaOutput));
            hardware.getRightBackMotor().setVelocity((posOutput) - ((isCounterClockwise ? -1 : 1) * thetaOutput));
        }

        navigateInALinearFashion(positions.get(positions.size() - 1));

        hardware.getLeftFrontMotor().setVelocity(0);
        hardware.getRightFrontMotor().setVelocity(0);
        hardware.getLeftBackMotor().setVelocity(0);
        hardware.getRightBackMotor().setVelocity(0);

        while (!hardware.opMode.isStopRequested())
        {
            hardware.opMode.telemetry.addData("Short by (absolute)", Math.sqrt(Math.pow(positions.get(positions.size() - 1).x - position.x, 2) + Math.sqrt(Math.pow(positions.get(positions.size() - 1).y - position.y, 2))));
            hardware.opMode.telemetry.addData("Short by (ratio)", Math.sqrt(Math.pow(positions.get(positions.size() - 1).x - position.x, 2) + Math.sqrt(Math.pow(positions.get(positions.size() - 1).y - position.y, 2))) / spline.splineDistance);
            hardware.opMode.telemetry.update();
        }
    }
}
