package com.chsrobotics.ftccore.engine.localization.localizer;

import com.chsrobotics.ftccore.geometry.Position;
import com.chsrobotics.ftccore.hardware.HardwareManager;

public abstract class Localizer {
    public Position currentPosition = null;
    private Position lastPosition = null;

    public double currentTime = 0.0;
    private double lastTime = 0.0;

    public Localizer(Position initialState, HardwareManager hardware)
    {
        double systemTime = System.currentTimeMillis();

        currentPosition = initialState;
        lastPosition = initialState;

        currentTime = systemTime;
        lastTime = systemTime;
    }

    public Position getRobotPosition() {
        return null;
    }

    public Position getRobotPosition(Position previousPosition) {
        return null;
    }

    public void updateRobotPosition(Position pos) {

    }
}