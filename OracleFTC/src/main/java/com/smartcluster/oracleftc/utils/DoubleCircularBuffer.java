// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.smartcluster.oracleftc.utils;

import java.util.Arrays;
import java.util.RandomAccess;

public class DoubleCircularBuffer implements RandomAccess {
    private double[] data;
    private int front;
    private int size;

    public DoubleCircularBuffer(int capacity) {
        data = new double[capacity];
        Arrays.fill(data, 0.0);
    }

    public int size() {
        return size;
    }


    public double getFirst() {
        return data[front];
    }


    public double getLast() {
        // If there are no elements in the buffer, do nothing
        if (size == 0) {
            return 0.0;
        }

        return data[(front + size - 1) % data.length];
    }


    public void addFirst(double value) {
        if (data.length == 0) {
            return;
        }

        front = moduloDec(front);

        data[front] = value;

        if (size < data.length) {
            size++;
        }
    }


    public void addLast(double value) {
        if (data.length == 0) {
            return;
        }

        data[(front + size) % data.length] = value;

        if (size < data.length) {
            size++;
        } else {
            // Increment front if buffer is full to maintain size
            front = moduloInc(front);
        }
    }

    public double removeFirst() {
        // If there are no elements in the buffer, do nothing
        if (size == 0) {
            return 0.0;
        }

        double temp = data[front];
        front = moduloInc(front);
        size--;
        return temp;
    }

    public double removeLast() {
        // If there are no elements in the buffer, do nothing
        if (size == 0) {
            return 0.0;
        }

        size--;
        return data[(front + size) % data.length];
    }

    public void resize(int size) {
        double[] newBuffer = new double[size];
        this.size = Math.min(this.size, size);
        for (int i = 0; i < this.size; i++) {
            newBuffer[i] = data[(front + i) % data.length];
        }
        data = newBuffer;
        front = 0;
    }

    public void clear() {
        Arrays.fill(data, 0.0);
        front = 0;
        size = 0;
    }

    public double get(int index) {
        return data[(front + index) % data.length];
    }


    private int moduloInc(int index) {
        return (index + 1) % data.length;
    }

    private int moduloDec(int index) {
        if (index == 0) {
            return data.length - 1;
        } else {
            return index - 1;
        }
    }
}