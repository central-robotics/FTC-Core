package com.chsrobotics.ftccore.engine.navigation.control;

import com.chsrobotics.ftccore.pipeline.Pipeline;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.chsrobotics.ftccore.geometry.Position;


public class PID {
    private final double kP;
    private final double kI;
    private final double kD;
    private double errorSum;
    private double lastError;

    public PID(PIDCoefficients coeffs)
    {
        kP = coeffs.p;
        kI = coeffs.i;
        kD = coeffs.d;
    }

    public double getOutput(double error, double speed) {
        errorSum += error * Pipeline.time.seconds();

        double derivative = (error - lastError) / Pipeline.time.seconds();

        lastError = error;

        return (kP * error) + (kI * errorSum) + (kD * derivative);
    }

    public double getSlope(Position target, Position position)
    {
        return (target.y - position.y) / (target.x - position.x);
    }

    public void resetSum() {
        errorSum = 0;
        lastError = 0;
    }
}
