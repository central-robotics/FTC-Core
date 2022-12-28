package com.chsrobotics.ftccore.engine.navigation.control;

import org.apache.commons.math3.analysis.polynomials.PolynomialSplineFunction;

public class ParametricSpline {
    public PolynomialSplineFunction xSpline;
    public PolynomialSplineFunction ySpline;
    private final PolynomialSplineFunction xPrimet;
    private final PolynomialSplineFunction yPrimet;
    public final double splineDistance;
    public final double[] secantDistances;

    public ParametricSpline(PolynomialSplineFunction f0, PolynomialSplineFunction f1, double dist, double[] distances)
    {

        xSpline = f0;
        ySpline = f1;
        xPrimet = xSpline.polynomialSplineDerivative();
        yPrimet = ySpline.polynomialSplineDerivative();
        splineDistance = dist;
        secantDistances = distances;
    }

    public double getDerivative(double t)
    {
        return yPrimet.value(t) / xPrimet.value(t);
    }

    public double getCurvature(double t)
    {
        return (Math.abs((xPrimet.value(t)*yPrimet.polynomialSplineDerivative().value(t))
                - (yPrimet.value(t) * xPrimet.derivative().value(t))))/(Math.pow(Math.pow(xPrimet.value(t), 2) + Math.pow(yPrimet.value(t), 2), 3/2d));
    }
}
