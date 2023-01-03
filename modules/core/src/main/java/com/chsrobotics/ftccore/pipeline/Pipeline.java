package com.chsrobotics.ftccore.pipeline;

import com.chsrobotics.ftccore.actions.Action;
import com.chsrobotics.ftccore.actions.ContinuousAction;
import com.chsrobotics.ftccore.actions.SetPrecisionAction;
import com.chsrobotics.ftccore.engine.localization.LocalizationEngine;
import com.chsrobotics.ftccore.engine.navigation.NavigationEngine;
import com.chsrobotics.ftccore.engine.navigation.path.MotionProfile;
import com.chsrobotics.ftccore.engine.navigation.path.Path;
import com.chsrobotics.ftccore.engine.navigation.path.PrecisionMode;
import com.chsrobotics.ftccore.geometry.Position;
import com.chsrobotics.ftccore.hardware.HardwareManager;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.ArrayList;

public class Pipeline {

    private HardwareManager manager;
    private ArrayList<PipelineStep> steps;
    private ArrayList<ContinuousAction> continuousActions;
    private final double conversion = Math.PI / 180;
    public final LocalizationEngine localization;

    public static ElapsedTime time = new ElapsedTime();
    private static Pipeline INSTANCE;


    public static Pipeline getInstance() {
        return INSTANCE;
    }

    private Pipeline(HardwareManager manager, ArrayList<PipelineStep> steps, ArrayList<ContinuousAction> continuousActions) {
        INSTANCE = this;
        this.manager = manager;
        this.steps = steps;
        this.continuousActions = continuousActions;
        this.localization = new LocalizationEngine(manager);

        time.startTime();
    }

    public void runContinuousActions() {
        for (ContinuousAction action : continuousActions) {
            action.execute();
        }
    }

    public void execute()
    {
        NavigationEngine navigationEngine = new NavigationEngine(localization, manager);

        for (PipelineStep step : steps)
        {
            if (manager.opMode.isStopRequested())
            {
                break;
            }
            if (step.type == PipelineStep.StepType.NAVIGATION)
            {
                assert step.path != null;
                if (step.path.isCurved) {
                    navigationEngine.navigateInANonLinearFashion(step.path.positions);
                    continue;
                }
                for (Position dest : step.path.positions) {
                    navigationEngine.linearController.resetSum();
                    navigationEngine.rotationController.resetSum();
                    if (step.path.profile != null && manager.profileCtrler != null)
                        manager.profileCtrler.resetSum();
                    if (manager.useDegrees) {
                        dest.t *= conversion;
                    }
                    if (manager.opMode.isStopRequested())
                    {
                        break;
                    }

                    if (step.path.profile != null)
                    {
                        step.path.profile.calculateProfile(localization.currentPosition, dest);
                        time.reset();
                    }

                    while (!navigationEngine.isTargetReached(dest) && !manager.opMode.isStopRequested()) {
                        navigationEngine.navigateInALinearFashion(dest, step.path.profile);
                        runContinuousActions();
                    }
                }
            } else if (step.type == PipelineStep.StepType.ACTION)
            {
                assert step.action != null;
                step.action.execute();
            } else if (step.type == PipelineStep.StepType.STOP)
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
                steps.add(new PipelineStep(PipelineStep.StepType.STOP));

            return this;
        }

        public Builder addLinearPath(Position... positions) {
            steps.add(new PipelineStep(Path.linear(positions)));

            return this;
        }

        public Builder addLinearPath(MotionProfile profile, Position... positions) {
            steps.add(new PipelineStep(Path.linear(profile, positions)));

            return this;
        }

        public Builder addLinearPath(MotionProfile profile, boolean continuous, Position... positions) {
            steps.add(new PipelineStep(Path.linear(profile, positions)));

            if (!continuous)
                steps.add(new PipelineStep(PipelineStep.StepType.STOP));

            return this;
        }
        public Builder addLinearPath(PrecisionMode precisionMode, Position... positions) {
            steps.add(new PipelineStep(new SetPrecisionAction(manager, precisionMode)));
            steps.add(new PipelineStep(Path.linear(positions)));
            steps.add(new PipelineStep(new SetPrecisionAction(manager, PrecisionMode.MEDIUM)));
            return this;
        }
        public Builder addLinearPath(PrecisionMode precisionMode, boolean continuous, Position... positions) {
            steps.add(new PipelineStep(new SetPrecisionAction(manager, precisionMode)));
            steps.add(new PipelineStep(Path.linear(positions)));
            steps.add(new PipelineStep(new SetPrecisionAction(manager, PrecisionMode.MEDIUM)));

            if (!continuous)
                steps.add(new PipelineStep(PipelineStep.StepType.STOP));

            return this;
        }

        public Builder addLinearPath(PrecisionMode precisionMode, MotionProfile profile, Position... positions) {
            steps.add(new PipelineStep(new SetPrecisionAction(manager, precisionMode)));
            steps.add(new PipelineStep(Path.linear(profile, positions)));
            steps.add(new PipelineStep(new SetPrecisionAction(manager, PrecisionMode.MEDIUM)));
            return this;
        }

        public Builder addLinearPath(PrecisionMode precisionMode, MotionProfile profile, boolean continuous, Position... positions) {
            steps.add(new PipelineStep(new SetPrecisionAction(manager, precisionMode)));
            steps.add(new PipelineStep(Path.linear(profile, positions)));
            steps.add(new PipelineStep(new SetPrecisionAction(manager, PrecisionMode.MEDIUM)));

            if (!continuous)
                steps.add(new PipelineStep(PipelineStep.StepType.STOP));

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
