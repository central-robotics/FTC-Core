package com.chsrobotics.ftccore.engine.localization.localizer;

import com.chsrobotics.ftccore.geometry.Position;
import com.chsrobotics.ftccore.hardware.HardwareManager;

import org.apache.commons.math3.linear.ArrayRealVector;
import org.apache.commons.math3.linear.RealVector;

public class IMULocalizer extends Localizer {

    /*
    IMU localizer uses the accelerometers on the robot to track position. This should never be used standalone, but complements
    encoder and odometry localization by accounting for slippage. Drift with IMU localization is inevitable.
    */

    public IMULocalizer(Position initialState, HardwareManager hardware) {
        super(initialState, hardware);
    }

    @Override
    public Position getRobotPosition() {
        return super.getRobotPosition();
    }

    @Override
    public void updateRobotPosition(Position pos) {

    }
}
