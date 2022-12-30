package com.chsrobotics.ftccore.pipeline;

import com.chsrobotics.ftccore.actions.Action;
import com.chsrobotics.ftccore.engine.navigation.path.Path;

public class PipelineStep {

    public final StepType type;
    public final Path path;
    public final Action action;

    public PipelineStep(Path path)
    {
        type = StepType.NAVIGATION;
        this.path = path;
        this.action = null;
    }

    public PipelineStep(Action action)
    {
        type = StepType.ACTION;
        this.action = action;
        this.path = null;
    }

    public PipelineStep(StepType type)
    {
        if (type == StepType.STOP) {
            this.path = null;
            this.action = null;
            this.type = type;
        } else {
            this.path = null;
            this.action = null;
            this.type = null;
        }
    }

    public enum StepType
    {
        NAVIGATION,
        ACTION,
        STOP
    }
}
