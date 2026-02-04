package com.smartcluster.oracleftc.math

import org.junit.Assert.assertArrayEquals
import org.junit.Assert.assertEquals
import org.junit.Test
import kotlin.math.pow

class TestParam;
class DualNumTest {
    @Test
    fun testAddition() {
        val a = DualNum<TestParam>(1.0, 2.0)
        val b = DualNum<TestParam>(3.0, 4.0)
        assertEquals(DualNum<TestParam>(4.0, 6.0), a + b)
    }

    @Test
    fun testMultiplication() {
        val a = DualNum<TestParam>(2.0, 1.0)
        val b = DualNum<TestParam>(3.0, 1.0)
        assertEquals(DualNum<TestParam>(6.0, 5.0, 2.0), a * b)
    }

    @Test
    fun testDivision() {
        val a = DualNum<TestParam>(6.0, 2.0)
        val b = DualNum<TestParam>(3.0)
        assertEquals(DualNum<TestParam>(2.0, 2.0 / 3.0), a / b)
    }
    @Test
    fun testHigherOrderDerivative() {
        val a = DualNum<TestParam>(1.0, 2.0, 3.0)
        val b = DualNum<TestParam>(2.0, 1.0, 0.5)
        assertEquals(DualNum<TestParam>(2.0, 5.0, 10.5, 12.0, 9.0), a * b)
    }

    @Test
    fun testZeroMultiplication() {
        val a = DualNum<TestParam>(0.0, 0.0, 0.0)
        val b = DualNum<TestParam>(3.0, 4.0, 5.0)
        assertEquals(DualNum<TestParam>(0.0, 0.0, 0.0,0.0,0.0), a * b)
    }

    @Test
    fun testReparameterization() {
        val a = DualNum<TestParam>(2.0, 3.0, 4.0)
        val oldParam = DualNum<TestParam>(1.0, 2.0, 3.0)
        assertEquals(DualNum<TestParam>(2.0, 6.0, 25.0), a.reparam(oldParam))
    }
    @Test
    fun testCos() {
        val a = DualNum<TestParam>(0.0, 1.0)
        assertEquals(DualNum<TestParam>(1.0, -0.0), a.cos())
    }

    @Test
    fun testSin() {
        val a = DualNum<TestParam>(0.0, 1.0)
        assertEquals(DualNum<TestParam>(0.0, 1.0), a.sin())
    }

    @Test
    fun testSqrt() {
        val a = DualNum<TestParam>(4.0, 2.0)
        assertEquals(DualNum<TestParam>(2.0, 0.5), a.sqrt())
    }
}