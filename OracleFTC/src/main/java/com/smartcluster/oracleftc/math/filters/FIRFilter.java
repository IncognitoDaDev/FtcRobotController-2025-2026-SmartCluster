package com.smartcluster.oracleftc.math.filters;

import java.util.Arrays;

public class FIRFilter {

    private final double[] filterTaps;
    private final double[] history;
    private int lastIndex;
    public FIRFilter(double[] filterTaps)
    {
        this.filterTaps=filterTaps;
        history = new double[filterTaps.length];
    }

    public double update(double measurement)
    {
        history[lastIndex++]=measurement;
        if(lastIndex ==filterTaps.length)
        {
            lastIndex =0;
        }

        double acc = 0;
        int index=lastIndex;
        for (double filterTap : filterTaps) {
            index = index != 0 ? index - 1 : filterTaps.length - 1;
            acc += history[index] * filterTap;
        }
        return acc;
    }

    public void reset()
    {
        Arrays.fill(history, 0);
        lastIndex=0;
    }
}
