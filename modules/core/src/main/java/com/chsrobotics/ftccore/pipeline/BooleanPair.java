package com.chsrobotics.ftccore.pipeline;

public class BooleanPair {
    public BooleanPair(Pipeline pipeline, EvaluableBoolean bool)
    {
        this.bool = bool;
        this.pipeline = pipeline;
    }

    public Pipeline pipeline;
    public EvaluableBoolean bool;
}
