package com.chsrobotics.ftccore.actions;

import com.chsrobotics.ftccore.hardware.HardwareManager;
import com.chsrobotics.ftccore.pipeline.Pipeline;

public abstract class Action {

    protected HardwareManager hardware;

    public Action(HardwareManager hardware) {
        this.hardware = hardware;
    }

    public abstract void execute();

    public boolean isContinuous() {
        return false;
    }

    public void setHardware(HardwareManager hardware) {
        this.hardware = hardware;
    }

}
