package com.smartcluster.oracleftc.math.filters;

import java.util.Arrays;

public class MovingAverageFilter {

    private final double[] history;
    private int head;
    private int count;
    private double sum;
    public MovingAverageFilter(int length)
    {
        history = new double[length];
    }

    public double update(double measurement)
    {
        int tail = (head+1)%history.length;
        sum = sum - history[tail]+measurement;
        head=tail;
        history[head]=measurement;
        if(count!=history.length) count++;
        return sum/count;

    }

    public void reset()
    {
        Arrays.fill(history,0);
        head =0;
    }

}
