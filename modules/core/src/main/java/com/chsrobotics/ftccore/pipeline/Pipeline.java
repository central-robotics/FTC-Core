package com.chsrobotics.ftccore.pipeline;

import com.chsrobotics.ftccore.actions.Action;
import com.chsrobotics.ftccore.actions.ContinuousAction;
import com.chsrobotics.ftccore.engine.localization.LocalizationEngine;
import com.chsrobotics.ftccore.engine.navigation.NavigationEngine;
import com.chsrobotics.ftccore.engine.navigation.path.Path;
import com.chsrobotics.ftccore.geometry.Position;
import com.chsrobotics.ftccore.hardware.HardwareManager;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import java.lang.reflect.Array;
import java.util.ArrayList;

public class Pipeline {

    private HardwareManager manager;
    private ArrayList<PipelineStep> steps;
    private ArrayList<ContinuousAction> continuousActions;

    private Pipeline(HardwareManager manager, ArrayList<PipelineStep> steps, ArrayList<ContinuousAction> continuousActions) {
        this.manager = manager;
        this.steps = steps;
        this.continuousActions = continuousActions;
    }

    private void runContinuousActions() {
        for (ContinuousAction action : continuousActions) {
            action.execute();
        }
    }

    public void execute()
    {
        LocalizationEngine localization = new LocalizationEngine(manager);
        NavigationEngine navigationEngine = new NavigationEngine(localization, manager);

        for (PipelineStep step : steps)
        {
            if (step.type == PipelineStep.StepType.navigation)
            {
                assert step.path != null;
                if (!step.path.isCurved) {
                    for (Position dest : step.path.positions) {
                        while (!navigationEngine.isTargetReached(dest) && !manager.opMode.isStopRequested()) {
                            navigationEngine.navigateInALinearFashion(dest);
                            runContinuousActions();
                        }
                    }
                } else
                {
                    navigationEngine.navigateInANonLinearFashion(step.path.positions);
                }
            } else if (step.type == PipelineStep.StepType.action)
            {
                assert step.action != null;
                step.action.execute();
            } else if (step.type == PipelineStep.StepType.stop)
            {
                for (DcMotorEx motor : manager.driveMotors)
                    motor.setPower(0);
            }
        }
    }

    public static class Builder {

        private final HardwareManager manager;
        private final ArrayList<PipelineStep> steps;
        private final ArrayList<ContinuousAction> continuousActions;

        public Builder(HardwareManager manager) {
            this.manager = manager;
            steps = new ArrayList<>();
            continuousActions = new ArrayList<>();
        }

        public Builder addAction(Action action) {
            steps.add(new PipelineStep(action));
            return this;
        }

        public Builder addCurvedPath(Position... positions) {
            steps.add(new PipelineStep(Path.curved(positions)));
            return this;
        }

        public Builder addLinearPath(boolean continuous, Position... positions) {
            steps.add(new PipelineStep(Path.linear(positions)));

            if (!continuous)
                steps.add(new PipelineStep(PipelineStep.StepType.stop));

            return this;
        }

        public Builder addLinearPath(Position... positions) {
            steps.add(new PipelineStep(Path.linear(positions)));

            return this;
        }

        public Builder addContinuousAction(ContinuousAction continuousAction)
        {
            continuousActions.add(continuousAction);
            return this;
        }

        public Pipeline build() {
            return new Pipeline(manager, steps, continuousActions);
        }
    }

}
