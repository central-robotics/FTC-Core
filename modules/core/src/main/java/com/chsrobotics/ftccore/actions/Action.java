package com.chsrobotics.ftccore.actions;

import com.chsrobotics.ftccore.hardware.HardwareManager;
import com.chsrobotics.ftccore.pipeline.Pipeline;

public abstract class Action {

    protected Pipeline pipeline;
    protected HardwareManager hardware;

    public Action() {

    }

    public Action(HardwareManager hardware, Pipeline pipeline) {
        this.hardware = hardware;
        this.pipeline = pipeline;
    }

    public abstract void execute();

    public boolean isContinuous() {
        return false;
    }

    public void setPipeline(Pipeline pipeline) {
        this.pipeline = pipeline;
    }

    public void setHardware(HardwareManager hardware) {
        this.hardware = hardware;
    }

}
