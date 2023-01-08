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
    private double lastTimestamp;
    private double windupCap = 0;

    public PID(PIDCoefficients coeffs)
    {
        kP = coeffs.p;
        kI = coeffs.i;
        kD = coeffs.d;
    }

    public PID(PIDCoefficients coeffs, double windupCap)
    {
        kP = coeffs.p;
        kI = coeffs.i;
        kD = coeffs.d;

        this.windupCap = windupCap;
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

        errorSum += error * period;

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
