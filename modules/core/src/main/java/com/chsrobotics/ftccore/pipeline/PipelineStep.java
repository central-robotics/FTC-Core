package com.chsrobotics.ftccore.pipeline;

import com.chsrobotics.ftccore.actions.Action;
import com.chsrobotics.ftccore.engine.navigation.path.Path;

public class PipelineStep {

    public final StepType type;
    public final Path path;
    public final Action action;
    public final BooleanPair boolPair;

    public PipelineStep(Path path)
    {
        type = StepType.navigation;
        this.path = path;
        this.action = null;
        this.boolPair = null;
    }

    public PipelineStep(Action action)
    {
        type = StepType.action;
        this.action = action;
        this.path = null;
        this.boolPair = null;
    }
    public PipelineStep(BooleanPair pair)
    {
        type = StepType.boolPair;
        this.boolPair = pair;
        this.action = null;
        this.path = null;
    }

    public enum StepType
    {
        navigation,
        action,
        boolPair
    }
}
