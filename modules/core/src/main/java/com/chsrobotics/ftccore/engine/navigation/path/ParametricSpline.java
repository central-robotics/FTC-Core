package com.chsrobotics.ftccore.engine.navigation.path;

import org.apache.commons.math3.analysis.polynomials.PolynomialSplineFunction;

public class ParametricSpline {
    public PolynomialSplineFunction xSpline;
    public PolynomialSplineFunction ySpline;
    public final PolynomialSplineFunction dx;
    public final PolynomialSplineFunction dy;
    public final double splineDistance;
    public final double[] secantDistances;
    public final double arcLength;

    public ParametricSpline(PolynomialSplineFunction f0, PolynomialSplineFunction f1, double dist, double[] distances, double arcLength)
    {

        xSpline = f0;
        ySpline = f1;
        dx = xSpline.polynomialSplineDerivative();
        dy = ySpline.polynomialSplineDerivative();
        splineDistance = dist;
        secantDistances = distances;
        this.arcLength = arcLength;
    }

    public double getDerivative(double t)
    {
        return dy.value(t) / dx.value(t);
    }

    public double getCurvature(double t)
    {
        return (Math.abs((dx.value(t)* dy.polynomialSplineDerivative().value(t))
                - (dy.value(t) * dx.derivative().value(t))))/(Math.pow(Math.pow(dx.value(t), 2) + Math.pow(dy.value(t), 2), 3d/2));
    }

    public double getT(double distAlongCurve) {
        return distAlongCurve / arcLength;
    }
}
