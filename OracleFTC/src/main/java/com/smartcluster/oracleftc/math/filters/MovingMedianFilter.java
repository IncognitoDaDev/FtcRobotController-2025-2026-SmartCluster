package com.smartcluster.oracleftc.math.filters;

import com.smartcluster.oracleftc.utils.DoubleCircularBuffer;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

import java.util.*;

public class MovingMedianFilter {
    private final DoubleCircularBuffer valueBuffer;
    private final List<Double> sortedValues;
    private final int length;

    /**
     * Creates a new MedianFilter.
     *
     * @param length The number of samples in the moving window.
     */
    public MovingMedianFilter(int length) {
        // Circular buffer of values currently in the window, ordered by time
        valueBuffer = new DoubleCircularBuffer(length);
        // List of values currently in the window, ordered by value
        sortedValues = new ArrayList<>(length);
        // Size of rolling window
        this.length = length;
    }

    /**
     * Calculates the moving-window median for the next value of the input stream.
     *
     * @param measurement The next input value.
     * @return The median of the moving window, updated to include the next value.
     */
    public double update(double measurement) {
        // Remove the oldest value if buffer is full
        if (valueBuffer.isFull()) {
            double oldestValue = valueBuffer.removeLast();
            int removeIndex = Collections.binarySearch(sortedValues, oldestValue);
            if (removeIndex >= 0) {
                sortedValues.remove(removeIndex);
            }
        }

        // Add new value to the buffer
        valueBuffer.addFirst(measurement);

        // Insert new value into sorted list
        int index = Collections.binarySearch(sortedValues, measurement);
        if (index < 0) {
            index = -(index + 1);
        }
        sortedValues.add(index, measurement);

        // Get the median
        int curSize = sortedValues.size();
        if (curSize % 2 != 0) {
            return sortedValues.get(curSize / 2);
        } else {
            return (sortedValues.get(curSize / 2 - 1) + sortedValues.get(curSize / 2)) / 2.0;
        }
    }

    /**
     * Returns the last value calculated by the MedianFilter.
     *
     * @return The last value.
     */
    public double lastValue() {
        if (valueBuffer.isEmpty()) {
            throw new NoSuchElementException("No values in filter.");
        }
        return valueBuffer.getFirst();
    }

    /** Resets the filter, clearing the window of all elements. */
    public void reset() {
        sortedValues.clear();
        valueBuffer.clear();
    }
}
