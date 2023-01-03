package com.chsrobotics.ftccore.engine.navigation.path;

public class Tolerances {

    public double linear;
    public double rotational;

    public Tolerances(double linearTolerance, double rotationalTolerance) {
        this.linear = linearTolerance;
        this.rotational = rotationalTolerance;
    }

}
