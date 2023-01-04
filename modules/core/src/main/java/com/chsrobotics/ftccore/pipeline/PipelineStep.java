package com.chsrobotics.ftccore.pipeline;

import com.chsrobotics.ftccore.actions.Action;
import com.chsrobotics.ftccore.engine.navigation.path.Path;
import com.qualcomm.robotcore.hardware.PIDCoefficients;

public class PipelineStep {

    public final StepType type;
    public final Path path;
    public final Action action;
    public final PIDCoefficients coeffs;

    public PipelineStep(Path path)
    {
        type = StepType.NAVIGATION;
        this.path = path;
        this.action = null;
        this.coeffs = null;
    }

    public PipelineStep(Action action)
    {
        type = StepType.ACTION;
        this.action = action;
        this.path = null;
        this.coeffs = null;
    }

    public PipelineStep(PIDCoefficients coeffs)
    {
        type = StepType.CHANGE_PID;
        this.coeffs = coeffs;
        this.path = null;
        this.action = null;
    }

    public PipelineStep(StepType type)
    {
        if (type == StepType.STOP) {
            this.path = null;
            this.action = null;
            this.coeffs = null;
            this.type = type;
        } else {
            this.path = null;
            this.action = null;
            this.type = null;
            this.coeffs = null;
        }
    }

    public enum StepType
    {
        NAVIGATION,
        ACTION,
        STOP,
        CHANGE_PID
    }
}
