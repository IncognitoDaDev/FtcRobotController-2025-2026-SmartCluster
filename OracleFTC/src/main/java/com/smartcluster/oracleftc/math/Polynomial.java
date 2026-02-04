package com.smartcluster.oracleftc.math;

import java.util.Arrays;

public class Polynomial {

    public final double[] coefficients;
    public Polynomial(double... coefficients)
    {
        this.coefficients=coefficients;
    }

    public double evaluate(double t)
    {
        if(coefficients.length==0) return 0;
        double result = 0;
        for(int i=0;i<coefficients.length;i++)
        {
            result += coefficients[i];
            if(i!=coefficients.length-1) result*=t;
        }
        return result;
    }

    public Polynomial getDerivative()
    {
        double[] derivativeCoefficients = Arrays.copyOf(coefficients, coefficients.length-1);
        for(int i=0;i<coefficients.length-1;i++)
        {
            derivativeCoefficients[i]*=(coefficients.length-1-i);
        }
        return new Polynomial(derivativeCoefficients);
    }
}
