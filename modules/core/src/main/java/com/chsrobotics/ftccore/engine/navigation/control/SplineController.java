package com.chsrobotics.ftccore.engine.navigation.control;

import com.chsrobotics.ftccore.geometry.Position;

public class SplineController
{
    // Returns the position as a point on the curve with the given value t, between 0 and 1, 0 being the beginning of the curve, 1 being the end.
    // The function is split into 4 distinct terms, one for each point. Each term represents the component vector created by that point.
    // The sum of all these vectors returns the vector which describes the position based on t. The path of the vector over all values t between 0 and 1 describes the spline.

    private Position getPositionVector(Position p0, Position p1, Position p2, Position p3, double t)
    {
        double p0Vector = Math.pow(1-t, 3); // (1-t)(1-2t+t^2) = 1-2t+t^2-t+2t^2-t^3 = -t^3+3t^2-3t+1; f' = -3t^2+6t-3
        double p1Vector = 3 * Math.pow(1-t, 2) * t; // 3t(1-2t+t^2) = 3t-6t^2+3t^3; f' = 9t^2-12t+3
        double p2Vector = 3 * (1-t) * Math.pow(t, 2); // 3t^2-3t^3
        double p3Vector = Math.pow(t, 3);

        double xVector = (p0.x * p0Vector) +
                (p1.x * p1Vector) +
                (p2.x * p2Vector) +
                (p3.x * p3Vector);

        double yVector = (p0.y * p0Vector) +
                (p1.y * p1Vector) +
                (p2.y * p2Vector) +
                (p3.y * p3Vector);

        return new Position(xVector, yVector, 0);
    }

    // Returns the velocity (rate of change) as a vector, which will determine the orientation of the robots' movement.
    // This is accomplished by taking the first derivative of the position function above.
    // We ignore the magnitude of the vector so that we can control the speed by other means.

    public Position getVelocityVector(Position p0, Position p1, Position p2, Position p3, double t)
    {
        double p1Scalar = -3 * Math.pow(t, 2) + 4*t - 1;
        double xVector = -3*(p0.x * Math.pow(1-t, 2) + p1.x * p1Scalar + t * (3 * p2.x * t - 2 * p2.x - p3.x * t));
        double yVector = -3*(p0.y * Math.pow(1-t, 2) + p1.y * p1Scalar + t * (3 * p2.y * t - 2 * p2.y - p3.y * t));

        Position vector = new Position(xVector, yVector, 0);

        vector.x = vector.x == 0 ? 0.0001 : vector.x;

        return vector;
    }

    // Returns the total length of the curve. This is an approximation based on a set number (in this case 200) of secant lines placed along the curve.

    public double getArcLength(Position p0, Position p1, Position p2, Position p3)
    {

    }

    // Returns an approximation of the value t which when passed to the position function would return the robot's current position.
    // This is needed by the controller to be passed to the velocity function at each iteration of the control loop.

    public double getT(double distAlongCurve, double arcLength)
    {
        return distAlongCurve / arcLength;
    }
}
