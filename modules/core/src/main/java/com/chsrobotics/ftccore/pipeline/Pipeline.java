package com.chsrobotics.ftccore.pipeline;

import com.chsrobotics.ftccore.actions.Action;
import com.chsrobotics.ftccore.engine.localization.LocalizationEngine;
import com.chsrobotics.ftccore.engine.navigation.NavigationEngine;
import com.chsrobotics.ftccore.engine.navigation.path.Path;
import com.chsrobotics.ftccore.geometry.Position;
import com.chsrobotics.ftccore.hardware.HardwareManager;

import java.util.ArrayList;

public class Pipeline {

    private HardwareManager manager;
    private ArrayList<PipelineStep> steps;

    private Pipeline(HardwareManager manager, ArrayList<PipelineStep> steps) {
        this.manager = manager;
        this.steps = steps;
    }

    public void execute() {
        LocalizationEngine localization = new LocalizationEngine(manager);
        NavigationEngine navigationEngine = new NavigationEngine(localization, manager);

        for (PipelineStep step : steps)
        {
            if (step.type == PipelineStep.StepType.navigation)
            {
                assert step.path != null;
                if (!step.path.isCurved) {
                    for (int i = 0; i < step.path.positions.size(); i++) {
                        navigationEngine.navigateInALinearFashion(step.path.positions.get(i));
                    }
                } else
                {
                    navigationEngine.navigateInANonLinearFashion(step.path.positions);
                }
            } else
            {
                assert step.action != null;
                step.action.execute();
            }
        }
    }

    public static class Builder {

        private final HardwareManager manager;
        private final ArrayList<PipelineStep> steps;

        public Builder(HardwareManager manager) {
            this.manager = manager;
            steps = new ArrayList<>();
        }

        public Builder addAction(Action action) {
            steps.add(new PipelineStep(action));
            return this;
        }

        public Builder addCurvedPath(Position... positions) {
            steps.add(new PipelineStep(Path.curved(positions)));
            return this;
        }

        public Builder addLinearPath(Position... positions) {
            steps.add(new PipelineStep(Path.linear(positions)));
            return this;
        }

        public Pipeline build() {
            return new Pipeline(manager, steps);
        }
    }

}
