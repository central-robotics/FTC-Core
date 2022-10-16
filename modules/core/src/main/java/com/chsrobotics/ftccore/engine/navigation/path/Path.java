package com.chsrobotics.ftccore.engine.navigation.path;

import com.chsrobotics.ftccore.geometry.Position;

import java.util.Arrays;
import java.util.List;

public class Path {

    public List<Position> positions;
    public boolean isCurved = false;

    public Path(List<Position> positions) {
        this.positions = positions;
    }

    public Path(List<Position> positions, boolean isCurved) {
        this.positions = positions;
        this.isCurved = isCurved;
    }

    public static Path curved(Position... positions) {
        return new Path(Arrays.asList(positions), true);
    }

    public static Path linear(Position... positions) {
        return new Path(Arrays.asList(positions), false);
    }

}
