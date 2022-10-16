package com.chsrobotics.ftccore.engine.navigation.control;

import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.chsrobotics.ftccore.geometry.Position;


public class PID {
    private final double kP;
    private final double kI;
    private final double kD;
    private double errorSum;

    public PID(PIDCoefficients coeffs)
    {
        kP = coeffs.p;
        kI = coeffs.i;
        kD = coeffs.d;
    }

    public double getOutput(double error, double speed) {
        errorSum += error;
        return (kP * error) + (kI * errorSum) - (kD * speed);
    }

    public double getSlope(Position target, Position position)
    {
        return (target.y - position.y) / (target.x - position.x);
    }

    public void resetSum() {
        errorSum = 0;
    }
}
