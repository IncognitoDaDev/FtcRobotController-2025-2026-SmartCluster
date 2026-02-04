package com.smartcluster.oracleftc.hardware.wrappers;


import com.smartcluster.oracleftc.math.DualNum;
import com.smartcluster.oracleftc.math.Time;

public interface Encoder {
    enum Direction {
        FORWARD, REVERSE;

        public Direction inverted() {
            return this == FORWARD ? REVERSE : FORWARD;
        }

        public int factor()
        {
            return this == FORWARD ? 1 : -1;
        }
    }


    /**
     * Sets the logical direction in which this encoder counts.
     * @param direction the direction to set for this encoder
     *
     * @see #getDirection()
     */
    void setDirection(Direction direction);

    /**
     * Returns the current logical direction in which this encoder is set as operating.
     * @return the current logical direction in which this encoder is set as operating.
     * @see #setDirection(Direction)
     */
    Direction getDirection();

    /**
     * Returns the current reading of the encoder. The units for this reading,
     * that is, the number of ticks per revolution, are specific to the encoder in question,
     * and thus are not specified here.
     * @return the current reading of the encoder
     */
    DualNum<Time> getCurrentPosition();

    /**
     * Resets the encoder
     *
     */
    void reset();
    /**
     * Resets the encoder
     *
     */
    void reset(double position);
}
