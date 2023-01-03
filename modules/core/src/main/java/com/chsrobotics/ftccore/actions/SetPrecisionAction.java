package com.chsrobotics.ftccore.actions;

import com.chsrobotics.ftccore.engine.navigation.path.PrecisionMode;
import com.chsrobotics.ftccore.hardware.HardwareManager;

public class SetPrecisionAction extends Action {

    private HardwareManager hardware;
    private PrecisionMode precision;

    public SetPrecisionAction(HardwareManager hardware, PrecisionMode precisionMode) {
        super(hardware);
        this.hardware = hardware;
        this.precision = precisionMode;
    }

    @Override
    public void execute() {
        hardware.setPrecisionMode(precision);
    }

}
