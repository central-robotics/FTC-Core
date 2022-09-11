package com.chsrobotics.ftccore.utilities;

import com.chsrobotics.ftccore.geometry.Position;

public class MathUtil {
    public static double calculateDistance(Position p0, Position p1)
    {
        double x = Math.pow(p1.x - p0.x, 2);
        double y = Math.pow(p1.y - p0.y, 2);

        return Math.sqrt(x + y);
    }
}
