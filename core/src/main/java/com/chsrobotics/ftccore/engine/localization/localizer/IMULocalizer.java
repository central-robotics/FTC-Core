package com.chsrobotics.ftccore.engine.localization.localizer;

import com.chsrobotics.ftccore.geometry.Position;
import com.chsrobotics.ftccore.hardware.HardwareManager;

public class IMULocalizer extends Localizer {
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
