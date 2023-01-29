package com.chsrobotics.ftccore.engine.navigation.control;

import com.chsrobotics.ftccore.pipeline.Pipeline;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.chsrobotics.ftccore.geometry.Position;


public class PID {
    private final double kP;
    private final double kI;
    private final double kD;
    public double errorSum;
    private double lastError;
    private double lastTimestamp;
    private double windupCap = 0;

    public PID(PIDParams params)
    {
        kP = params.coefficients.p;
        kI = params.coefficients.i;
        kD = params.coefficients.d;
        windupCap = params.integralCap;
    }

    public double getOutput(double error, double speed) {
        double currentTime= (double) System.nanoTime() / 1E9;

        if (lastTimestamp == 0)
            lastTimestamp = currentTime;

        double period = currentTime - lastTimestamp;

        lastTimestamp = currentTime;

        double derivative = 0;

        if (Math.abs(period) > 1E-6) {
            derivative = (error - lastError) / period;
        } else {
            derivative = 0;
        }

        errorSum += error;

        if (errorSum > windupCap) {
            errorSum = windupCap;
        }
        if (errorSum < -windupCap) {
            errorSum = -windupCap;
        }

        lastError = error;

        return (kP * error) + (kI * errorSum) + (kD * derivative);
    }

    public double getSlope(Position target, Position position)
    {
        return (target.y - position.y) / (target.x - position.x);
    }

    public void capWindup(double cap)
    {
        windupCap = cap;
    }

    public void resetSum() {
        errorSum = 0;
        lastError = 0;
        lastTimestamp = 0;
    }
}
