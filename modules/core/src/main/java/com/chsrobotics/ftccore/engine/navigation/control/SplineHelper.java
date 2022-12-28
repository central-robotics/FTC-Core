package com.chsrobotics.ftccore.engine.navigation.control;

import org.apache.commons.math3.analysis.ParametricUnivariateFunction;
import org.apache.commons.math3.analysis.UnivariateFunction;
import org.apache.commons.math3.analysis.integration.BaseAbstractUnivariateIntegrator;
import org.apache.commons.math3.analysis.integration.RombergIntegrator;
import org.apache.commons.math3.analysis.integration.UnivariateIntegrator;
import org.apache.commons.math3.analysis.integration.gauss.GaussIntegrator;
import org.apache.commons.math3.analysis.interpolation.SplineInterpolator;
import org.apache.commons.math3.analysis.interpolation.UnivariateInterpolator;
import org.apache.commons.math3.analysis.polynomials.PolynomialSplineFunction;
import org.apache.commons.math3.ode.nonstiff.AdaptiveStepsizeIntegrator;

public class SplineHelper {


    public ParametricSpline computeSpline(double[] x, double[] y)
    {
        int n = x.length;
        double[] distances = new double[n];
        distances[0] = 0;
        double totalDist = 0;

        for (int i = 1; i < n; i++)
        {
            double dx = x[i] - x[i - 1];
            double dy = y[i] - y[i - 1];
            double dist = Math.sqrt(dx * dx + dy * dy);
            totalDist += dist;
            distances[i] = totalDist;
        }

        PolynomialSplineFunction f0 = new SplineInterpolator().interpolate(distances, x);
        PolynomialSplineFunction f1 = new SplineInterpolator().interpolate(distances, y);

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

        for (int i = 0; i < distances.length; i++)
        {
            double distance = distances[i];
            distances[i] = distance / totalDist;
        }

        f0 = new SplineInterpolator().interpolate(distances, xs);
        f1 = new SplineInterpolator().interpolate(distances, ys);

        return new ParametricSpline(f0, f1, totalDist, new double[1]);
    }
}
