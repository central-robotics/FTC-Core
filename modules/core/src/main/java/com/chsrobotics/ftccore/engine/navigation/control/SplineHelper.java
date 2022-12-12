package com.chsrobotics.ftccore.engine.navigation.control;

import org.apache.commons.math3.analysis.ParametricUnivariateFunction;
import org.apache.commons.math3.analysis.integration.RombergIntegrator;
import org.apache.commons.math3.analysis.integration.UnivariateIntegrator;
import org.apache.commons.math3.analysis.interpolation.SplineInterpolator;
import org.apache.commons.math3.analysis.polynomials.PolynomialSplineFunction;

public class SplineHelper {
    public ParametricSpline computeSpline(double[] x, double[] y)
    {
        double arcLength = 0;
        double[] t = new double[x.length];
        SplineInterpolator interp = new SplineInterpolator();


        t[0] = 0;

        for (int i = 1; i < x.length; i++)
        {
            arcLength += Math.sqrt( Math.pow(x[i] - x[i - 1], 2) + Math.pow(y[i] - y[i - 1], 2) );
        }

        for (int i = 1; i < x.length; i++)
        {
            t[i] = t[i-1] + (Math.sqrt( Math.pow(x[i] - x[i - 1], 2) + Math.pow(y[i] - y[i - 1], 2) )) / arcLength;
        }

        PolynomialSplineFunction fX = interp.interpolate(t, x);
        PolynomialSplineFunction fY = interp.interpolate(t, y);

        return new ParametricSpline(fX, fY, arcLength, new double[1]);
    }
}
