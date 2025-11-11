package com.smartcluster.oracleftc.math.filters;

public class LowPassFilter extends FIRFilter {

    /**
     * Create a low pass filter
     * <p>
     * High values of gain are smoother but have more phase lag, low values of gain allow more noise but
     * will respond faster to quick changes in the measured state.
     *
     * @param gain low pass filter gain
     */
    public LowPassFilter(double gain) {
        super(new double[] {1 - gain, gain});
        if (gain < 0 || gain > 1) {
            throw new IllegalArgumentException("LowPassFilter gain must be between 0 and 1");
        }
    }
}
