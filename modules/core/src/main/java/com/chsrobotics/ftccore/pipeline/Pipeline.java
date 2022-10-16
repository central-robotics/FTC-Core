package com.chsrobotics.ftccore.pipeline;

import com.chsrobotics.ftccore.engine.localization.LocalizationEngine;
import com.chsrobotics.ftccore.engine.navigation.NavigationEngine;
import com.chsrobotics.ftccore.engine.navigation.path.Path;
import com.chsrobotics.ftccore.geometry.Position;
import com.chsrobotics.ftccore.hardware.HardwareManager;

import java.util.ArrayList;
import java.util.List;

public class Pipeline {

    private HardwareManager manager;
    private List<Path> paths = new ArrayList<>();

    private Pipeline(HardwareManager manager, List<Path> paths) {
        this.manager = manager;
        this.paths = paths;
    }

    public void execute() {
        LocalizationEngine localization = new LocalizationEngine(manager);
        NavigationEngine navigationEngine = new NavigationEngine(localization, manager);
        for (Path path : paths)
        {
            if (!path.isCurved)
            {
                for (int i = 0; i < path.positions.size(); i++)
                {
                    navigationEngine.navigateInALinearFashion(path.positions.get(i));
                }
            }
        }
    }

    public static class Builder {

        private HardwareManager manager;
        private List<Path> paths = new ArrayList<>();

        public Builder(HardwareManager manager) {
            this.manager = manager;
        }

        public Builder addCurvedPath(Position... positions) {
            for (Position position : positions) {
                position.t *= Math.PI / 180;
            }
            paths.add(Path.curved(positions));
            return this;
        }

        public Builder addLinearPath(Position... positions) {
            for (Position position : positions) {
                position.t *= Math.PI / 180;
            }
            paths.add(Path.linear(positions));
            return this;
        }

        public Pipeline build() {
            return new Pipeline(manager, paths);
        }
    }

}
