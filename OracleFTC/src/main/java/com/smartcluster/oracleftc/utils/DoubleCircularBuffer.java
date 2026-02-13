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

    public boolean isFull() {
        return size == data.length;
    }

    public boolean isEmpty() {
        return size == 0;
    }

    public double getFirst() {
        if (isEmpty()) {
            throw new IllegalStateException("Buffer is empty.");
        }
        return data[front];
    }

    public double getLast() {
        if (isEmpty()) {
            throw new IllegalStateException("Buffer is empty.");
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
            front = moduloInc(front); // Move front when buffer is full
        }
    }

    public double removeFirst() {
        if (isEmpty()) {
            throw new IllegalStateException("Buffer is empty.");
        }

        double temp = data[front];
        front = moduloInc(front);
        size--;
        return temp;
    }

    public double removeLast() {
        if (isEmpty()) {
            throw new IllegalStateException("Buffer is empty.");
        }

        size--;
        return data[(front + size) % data.length];
    }

    public void resize(int newSize) {
        double[] newBuffer = new double[newSize];
        this.size = Math.min(this.size, newSize);
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
        if (index < 0 || index >= size) {
            throw new IndexOutOfBoundsException("Index out of bounds: " + index);
        }
        return data[(front + index) % data.length];
    }

    private int moduloInc(int index) {
        return (index + 1) % data.length;
    }

    private int moduloDec(int index) {
        return (index == 0) ? data.length - 1 : index - 1;
    }
}
