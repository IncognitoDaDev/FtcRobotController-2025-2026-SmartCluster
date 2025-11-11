package com.smartcluster.oracleftc.math.control;


// TODO: Rewrite using style guide
public class SlewRateLimiter {
    public double positiveRateLimit;
    public double negativeRateLimit;
    private double m_prevVal;
    private double m_prevTime;

    /**
     * Creates a new SlewRateLimiter with the given positive and negative rate limits and initial
     * value.
     *
     * @param positiveRateLimit The rate-of-change limit in the positive direction, in units per
     *     second. This is expected to be positive.
     * @param negativeRateLimit The rate-of-change limit in the negative direction, in units per
     *     second. This is expected to be negative.
     * @param initialValue The initial value of the input.
     */
    public SlewRateLimiter(double positiveRateLimit, double negativeRateLimit, double initialValue) {
        this.positiveRateLimit = positiveRateLimit;
        this.negativeRateLimit = negativeRateLimit;
        m_prevVal = initialValue;
        m_prevTime = System.nanoTime()/1E9;
    }

    /**
     * Creates a new SlewRateLimiter with the given positive rate limit and negative rate limit of
     * -rateLimit.
     *
     * @param rateLimit The rate-of-change limit, in units per second.
     */
    public SlewRateLimiter(double rateLimit) {
        this(rateLimit, -rateLimit, 0);
    }

    /**
     * Filters the input to limit its slew rate.
     *
     * @param input The input value whose slew rate is to be limited.
     * @return The filtered value, which will not change faster than the slew rate.
     */
    public double calculate(double input) {
        double currentTime = System.nanoTime()/1E9;
        double elapsedTime = currentTime - m_prevTime;
        m_prevVal += Math.max(negativeRateLimit * elapsedTime, Math.min(input - m_prevVal,positiveRateLimit * elapsedTime));
        m_prevTime = currentTime;
        return m_prevVal;
    }

    /**
     * Resets the slew rate limiter to the specified value; ignores the rate limit when doing so.
     *
     * @param value The value to reset to.
     */
    public void reset(double value) {
        m_prevVal = value;
        m_prevTime = System.nanoTime()/1E9;
    }
    public void reset()
    {
        m_prevVal=0;
        m_prevTime=System.nanoTime()/1E9;
    }
}