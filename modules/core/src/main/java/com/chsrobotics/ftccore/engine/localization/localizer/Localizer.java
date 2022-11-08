package com.chsrobotics.ftccore.engine.localization.localizer;

import com.chsrobotics.ftccore.geometry.Position;
import com.chsrobotics.ftccore.hardware.HardwareManager;

public abstract class Localizer {

    /*
    The localizer class is an abstract class that allows for multiple localization implementations.
    This provides access to localization to teams that don't have certain components on their robot.
    Teams that have access to multiple forms of localization are benefited in that these inputs are
    fused via a Kalman filter in the localization engine.
     */

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
