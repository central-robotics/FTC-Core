package com.chsrobotics.ftccore.actions;

import com.chsrobotics.ftccore.hardware.HardwareManager;
import com.chsrobotics.ftccore.pipeline.Pipeline;

public abstract class Action {

    private int index;
    private int priority;
    protected Pipeline pipeline;
    protected HardwareManager hardware;

    public Action() {
        index = -1;
        priority = -1;
    }

    public Action(HardwareManager hardware, Pipeline pipeline) {
        this.hardware = hardware;
        this.pipeline = pipeline;
        index = -1;
        priority = -1;
    }

    public Action(HardwareManager hardware, Pipeline pipeline, int index, int priority)
    {
        this.hardware = hardware;
        this.pipeline = pipeline;
        this.index = index;
        this.priority = priority;
    }

    public abstract void execute();

    public boolean isContinuous() {
        return false;
    }

    public int getIndex() {
        return index;
    }

    public void setIndex(int index) {
        this.index = index;
    }

    public int getPriority() {
        return priority;
    }

    public void setPriority(int priority) {
        this.priority = priority;
    }

    public void setPipeline(Pipeline pipeline) {
        this.pipeline = pipeline;
    }

    public void setHardware(HardwareManager hardware) {
        this.hardware = hardware;
    }

}
