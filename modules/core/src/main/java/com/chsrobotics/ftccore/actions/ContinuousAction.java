package com.chsrobotics.ftccore.actions;

import com.chsrobotics.ftccore.hardware.HardwareManager;
import com.chsrobotics.ftccore.pipeline.Pipeline;

public abstract class ContinuousAction extends Action {
    public ContinuousAction(HardwareManager hardware) {
        super(hardware);
    }

    public abstract void initialize();

    public boolean isFinished() {
        return true;
    }
    @Override
    public boolean isContinuous() {
        return true;
    }
}
