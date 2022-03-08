package com.chsrobotics.ftccore.engine.localization.localizer;

import com.chsrobotics.ftccore.geometry.Position;

public class EncoderLocalizer extends Localizer{

    public EncoderLocalizer(Position initialState) {
        super(initialState);
    }

    @Override
    public Position getRobotPosition() {
        return null;
    }

    @Override
    public void updateRobotPosition() {

    }
}
