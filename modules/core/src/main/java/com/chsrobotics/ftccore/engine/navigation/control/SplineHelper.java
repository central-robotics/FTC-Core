package com.chsrobotics.ftccore.engine.navigation.control;

import com.chsrobotics.ftccore.engine.navigation.path.ParametricSpline;
import com.chsrobotics.ftccore.geometry.Position;

import org.apache.commons.math3.analysis.interpolation.SplineInterpolator;
import org.apache.commons.math3.analysis.polynomials.PolynomialFunction;
import org.apache.commons.math3.analysis.polynomials.PolynomialSplineFunction;

import java.util.Arrays;
import java.util.List;

public class SplineHelper
{
    public ParametricSpline computeSpline(List<Position> points)
    {
        int n = points.size();
        double[] distances = new double[n];
        distances[0] = 0;
        double totalDist = 0;

        for (int i = 1; i < n; i++)
        {
            double dx = points.get(i).x - points.get(i-1).x;
            double dy = points.get(i).y - points.get(i-1).y;
            double dist = Math.sqrt(dx * dx + dy * dy);
            totalDist += dist;
            distances[i] = totalDist;
        }

        PolynomialSplineFunction f0 = new SplineInterpolator().interpolate(distances, );
        PolynomialSplineFunction f1 = new SplineInterpolator().interpolate(distances, );

        //We want a spline that has a more accurately calculated distance.
        double[] xs = new double[(int) totalDist + 1];
        double[] ys = new double[(int) totalDist + 1];

        for (int i = 0; i < totalDist; i++)
        {
            xs[i] = f0.value(i);
            ys[i] = f1.value(i);
        }

        n = xs.length;
        distances = new double[n];
        distances[0] = 0;
        totalDist = 0;

        for (int i = 1; i < n; i ++)
        {
            double dx = xs[i] - xs[i - 1];
            double dy = ys[i] - ys[i - 1];
            double dist = Math.sqrt(dx * dx + dy * dy);
            totalDist += dist;
            distances[i] = totalDist;
        }

        f0 = new SplineInterpolator().interpolate(distances, xs);
        f1 = new SplineInterpolator().interpolate(distances, ys);

        return new ParametricSpline(f0, f1, totalDist, distances);
    }
}
