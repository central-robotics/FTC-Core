package com.chsrobotics.ftccore.engine.navigation.path;

import com.chsrobotics.ftccore.geometry.Position;

import java.util.Arrays;
import java.util.List;

public class Path {

    public List<Position> positions;
    public boolean isCurved = false;

    public MotionProfile profile = null;

    public Path(List<Position> positions) {
        this.positions = positions;
    }

    public Path(List<Position> positions, boolean isCurved, MotionProfile profile) {
        this.positions = positions;
        this.isCurved = isCurved;
        this.profile = profile;
    }

    public static Path curved(MotionProfile profile, Position... positions) {
        return new Path(Arrays.asList(positions), true, profile);
    }

    public static Path curved(Position... positions) {
        return new Path(Arrays.asList(positions), true, null);
    }

    public static Path linear(MotionProfile profile, Position... positions) {
        return new Path(Arrays.asList(positions), false, profile);
    }

    public static Path linear(Position... positions) {
        return new Path(Arrays.asList(positions), false, null);
    }

}
