package com.smartcluster.oracleftc.math.filters;

import com.smartcluster.oracleftc.utils.DoubleCircularBuffer;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

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
        // Find insertion point for next value
        int index = Collections.binarySearch(sortedValues, measurement);

        // Deal with binarySearch behavior for element not found
        if (index < 0) {
            index = Math.abs(index + 1);
        }

        // Place value at proper insertion point
        sortedValues.add(index, measurement);

        int curSize = sortedValues.size();

        // If buffer is at max size, pop element off of end of circular buffer
        // and remove from ordered list
        if (curSize > length) {
            sortedValues.remove(valueBuffer.removeLast());
            --curSize;
        }

        // Add next value to circular buffer
        valueBuffer.addFirst(measurement);

        if (curSize % 2 != 0) {
            // If size is odd, return middle element of sorted list
            return sortedValues.get(curSize / 2);
        } else {
            // If size is even, return average of middle elements
            return (sortedValues.get(curSize / 2 - 1) + sortedValues.get(curSize / 2)) / 2.0;
        }
    }

    /**
     * Returns the last value calculated by the MedianFilter.
     *
     * @return The last value.
     */
    public double lastValue() {
        return valueBuffer.getFirst();
    }

    /** Resets the filter, clearing the window of all elements. */
    public void reset() {
        sortedValues.clear();
        valueBuffer.clear();
    }
}