package com.chsrobotics.ftccore.engine.navigation.control;

import com.qualcomm.robotcore.hardware.PIDCoefficients;

public class PIDParams {
    public PIDCoefficients coefficients;
    public double integralCap = 0;

    public PIDParams(double p, double i, double d, double cap)
    {
        coefficients = new PIDCoefficients(p, i, d);
        integralCap = cap;
    }

    public PIDParams(double p, double i, double d)
    {
        coefficients = new PIDCoefficients(p, i, d);
    }
}
