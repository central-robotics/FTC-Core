package com.chsrobotics.ftccore.engine.localization.localizer;

import com.chsrobotics.ftccore.geometry.Position;
import com.chsrobotics.ftccore.hardware.HardwareManager;

public class EncoderLocalizer extends Localizer{

    public EncoderLocalizer(Position initialState, HardwareManager hardware) {
        super(initialState, hardware);
    }

    @Override
    public Position getRobotPosition() {
        return null;
    }

    @Override
    public void updateRobotPosition(Position pos) {
    }
}
