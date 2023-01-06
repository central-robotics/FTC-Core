package com.chsrobotics.ftccore.geometry;

public class Position {
    public double x;
    public double y;
    public double t;
    public double time = 0.0;

    public Position() {

    }

    public Position(double x, double y, double t)
    {
        this.x = x;
        this.y = y;
        this.t = t;
    }

    public Position(double x, double y)
    {
        this.x = x;
        this.y = y;
    }

    public Position(double x, double y, double t, double secondsBeforeRotation)
    {
        this.x = x;
        this.y = y;
        this.t = t;
        this.time = secondsBeforeRotation;
    }
}
