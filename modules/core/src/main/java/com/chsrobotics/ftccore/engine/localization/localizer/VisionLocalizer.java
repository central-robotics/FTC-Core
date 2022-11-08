package com.chsrobotics.ftccore.engine.localization.localizer;

import com.chsrobotics.ftccore.geometry.Position;
import com.chsrobotics.ftccore.hardware.HardwareManager;

public class VisionLocalizer extends Localizer{

    /*
    Uses the T265 VSLAM camera for localization that accounts for slippage. Vuforia will also be introduced later.
    */

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
