package com.smartcluster.oracleftc.autonomous.pathing;

import com.qualcomm.robotcore.util.Range;
import com.smartcluster.oracleftc.math.Polynomial;
import com.smartcluster.oracleftc.math.Vector2d;

import java.util.ArrayList;
import java.util.List;

import Jama.EigenvalueDecomposition;
import Jama.Matrix;

public class BezierCurve {

    private final Polynomial px, pdx, pd2x, py, pdy, pd2y;

    public BezierCurve(Vector2d... controlPoints) {
        if (controlPoints.length != 4)
            throw new IllegalArgumentException("BezierCurve requires 4 control points");
        px = new Polynomial(controlPoints[3].x - 3 * controlPoints[2].x + 3 * controlPoints[1].x - controlPoints[0].x, 3 * (controlPoints[2].x - 2 * controlPoints[1].x + controlPoints[0].x), 3 * (controlPoints[1].x - controlPoints[0].x), controlPoints[0].x);
        pdx = px.getDerivative();
        pd2x = pdx.getDerivative();

        py = new Polynomial(controlPoints[3].y - 3 * controlPoints[2].y + 3 * controlPoints[1].y - controlPoints[0].y, 3 * (controlPoints[2].y - 2 * controlPoints[1].y + controlPoints[0].y), 3 * (controlPoints[1].y - controlPoints[0].y), controlPoints[0].y);
        pdy = py.getDerivative();
        pd2y = pdy.getDerivative();
    }

    public static List<Double> solveQuintic(double a, double b, double c, double d, double e, double f) {
        List<Double> roots = new ArrayList<>();

        // Normalize the polynomial by dividing by 'a'
        double[] coefficients = {b / a, c / a, d / a, e / a, f / a};

        // Construct the Companion Matrix
        double[][] matrixData = {{0, 0, 0, 0, -coefficients[4]}, {1, 0, 0, 0, -coefficients[3]}, {0, 1, 0, 0, -coefficients[2]}, {0, 0, 1, 0, -coefficients[1]}, {0, 0, 0, 1, -coefficients[0]}};
        Matrix companionMatrix = new Matrix(matrixData);

        // Compute Eigenvalues (roots)
        EigenvalueDecomposition eig = new EigenvalueDecomposition(companionMatrix);
        double[] realParts = eig.getRealEigenvalues();
        double[] imagParts = eig.getImagEigenvalues();

        // Filter real roots in range [0,1]
        for (int i = 0; i < realParts.length; i++) {
            if (imagParts[i] == 0 && realParts[i] >= 0 && realParts[i] <= 1) {
                roots.add(realParts[i]);
            }
        }

        return roots;
    }

    public Vector2d getPoint(double t) {
        t = Range.clip(t, 0, 1);
        return new Vector2d(px.evaluate(t), py.evaluate(t));
    }

    public Vector2d getDerivative(double t) {
        t = Range.clip(t, 0, 1);
        return new Vector2d(pdx.evaluate(t), pdy.evaluate(t));
    }

    public Vector2d getSecondDerivative(double t) {
        t = Range.clip(t, 0, 1);
        return new Vector2d(pd2x.evaluate(t), pd2y.evaluate(t));
    }


    public double getCurvature(double t) {
        t = Range.clip(t, 0, 1);
        Vector2d derivative = getDerivative(t);
        Vector2d secondDerivative = getSecondDerivative(t);

        if (derivative.norm() == 0) return 0;

        return derivative.cross(secondDerivative) / Math.pow(derivative.norm(), 3);
    }

    public double getClosestT(Vector2d point) {
        double ax = px.coefficients[3], bx = px.coefficients[2], cx = px.coefficients[1], dx = px.coefficients[0] - point.x;
        double ay = py.coefficients[3], by = py.coefficients[2], cy = py.coefficients[1], dy = py.coefficients[0] - point.y;

        double a = 6 * (ax * ax + ay * ay);
        double b = 10 * (ax * bx + ay * by);
        double c = 4 * (2 * (ax * cx + ay * cy) + bx * bx + by * by);
        double d = 6 * (ax * dx + bx * cx + ay * dy + by * cy);
        double e = 2 * (2 * (bx * dx + by * dy) + cx * cx + cy * cy);
        double f = 2 * (cx * dx + cy * dy);

        double da = 5 * a;
        double db = 4 * b;
        double dc = 3 * c;
        double dd = 2 * d;
        double de = e;

        Polynomial dPolynomial = new Polynomial(da, db, dc, dd, de);
        List<Double> roots = solveQuintic(a, b, c, d, e, f);

        double minDistance = Double.MAX_VALUE;
        double selectedRoot = 0;
        for (double root : roots) {
            double distanceToPoint = point.minus(getPoint(root)).norm();
            if (dPolynomial.evaluate(root) >= 0 && distanceToPoint < minDistance) {
                minDistance = distanceToPoint;
                selectedRoot = root;
            }
        }
        return selectedRoot;
    }

    public Vector2d getClosestPoint(Vector2d point)
    {
        return getPoint(getClosestT(point));
    }
}


