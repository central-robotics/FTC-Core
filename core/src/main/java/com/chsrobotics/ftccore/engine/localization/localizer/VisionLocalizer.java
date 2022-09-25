package com.chsrobotics.ftccore.engine.localization.localizer;

import com.chsrobotics.ftccore.geometry.Position;
import com.chsrobotics.ftccore.hardware.HardwareManager;

public class VisionLocalizer extends Localizer{
    public VisionLocalizer(Position initialState, HardwareManager hardware) {
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
