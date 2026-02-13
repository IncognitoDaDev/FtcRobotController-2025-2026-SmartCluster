package com.smartcluster.oracleftc.math

import kotlin.math.atan
import kotlin.math.max
import kotlin.math.min




class Time;
class DualNum<Param>(private vararg val values: Double) {
    operator fun get(i: Int) = if (i < values.size) values[i] else 0.0
    operator fun set(i: Int, value: Double) {
        values[i]=value
    }

    @Suppress("MemberVisibilityCanBePrivate")
    val size: Int
        get() = values.size

    operator fun plus(d: DualNum<Param>): DualNum<Param> {
        val out = DoubleArray(max(d.size, size))
        for (i in out.indices) {
            out[i] = this[i] + d[i]
        }
        return DualNum(*out)
    }

    operator fun plus(c: Double): DualNum<Param> {
        val out = DualNum<Param>(*values)
        out[0]+=c
        return out
    }




    operator fun minus(d: DualNum<Param>): DualNum<Param> {
        val out = DoubleArray(max(d.size, size))
        for (i in out.indices) {
            out[i] = this[i] - d[i]
        }
        return DualNum(*out)
    }

    operator fun unaryMinus(): DualNum<Param> {
        val out = DoubleArray(size)
        for (i in out.indices) {
            out[i] = -this[i]
        }
        return DualNum(*out)
    }

    operator fun times(d: DualNum<Param>): DualNum<Param> {
        val out = DoubleArray(size + d.size - 1)
        for (i in values.indices) {
            for (j in d.values.indices) {
                out[i + j] += this[i] * d[j] * choose(i + j, i)
            }
        }
        return DualNum(*out)
    }

    operator fun times(c: Double): DualNum<Param> {
        val out = DoubleArray(size)
        for (i in out.indices) {
            out[i] = values[i] * c
        }

        return DualNum(*out)
    }

    operator fun div(d: DualNum<Param>): DualNum<Param> {
        if (d[0] == 0.0) throw ArithmeticException("Division by zero.")
        val out = DoubleArray(size)
        out[0] = this[0] / d[0]
        for (n in 1 until size) {
            var sum = 0.0
            for (k in 1..n) {
                sum += d[k] * out[n - k]
            }
            out[n] = (this[n] - sum) / d[0]
        }
        return DualNum(*out)
    }

    operator fun div(c: Double): DualNum<Param> {
        val out = DoubleArray(size)
        for (i in out.indices) {
            out[i] = values[i] / c
        }

        return DualNum(*out)
    }

    fun <NewParam> reparam(oldParam: DualNum<NewParam>): DualNum<NewParam> {
        val outSize = min(this.size, oldParam.size)
        val outValues = DoubleArray(outSize) { 0.0 }
        outValues[0] = this[0]
        if (outSize > 1) {
            outValues[1] = this[1] * oldParam[1]
        }
        if (outSize > 2) {
            for (n in 2 until outSize) {
                var sum = 0.0
                for (k in 1..n) {
                    sum += this[k] * bellPolynomial(n, k, oldParam)
                }
                outValues[n] = sum
            }
        }
        return DualNum(*outValues)
    }

    fun drop(n: Int) = DualNum<Param>(*DoubleArray(size - n) { values[it + n] })

    override fun equals(other: Any?): Boolean = other is DualNum<*> && values.contentEquals(other.values)
    override fun hashCode(): Int = values.contentHashCode()
    override fun toString(): String = "DualNum(${values.contentToString()})"
}

fun <Param>cos(d: DualNum<Param>): DualNum<Param> {
    val out = DoubleArray(d.size)
    out[0] = kotlin.math.cos(d[0])
    if (d.size > 1) {
        out[1] = -kotlin.math.sin(d[0]) * d[1]
        for (n in 2 until d.size) {
            out[n] = -d[n - 1] * kotlin.math.sin(d[0]) - (n - 1) * out[n - 2]
        }
    }
    return DualNum(*out)
}

fun <Param>sin(d: DualNum<Param>): DualNum<Param> {
    val out = DoubleArray(d.size)
    out[0] = kotlin.math.sin(d[0])
    if (d.size > 1) {
        out[1] = kotlin.math.cos(d[0]) * d[1]
        for (n in 2 until d.size) {
            out[n] = d[n - 1] * kotlin.math.cos(d[0]) - (n - 1) * out[n - 2]
        }
    }
    return DualNum(*out)
}

fun <Param>atan(d: DualNum<Param>): DualNum<Param> {
    val out = DoubleArray(d.size)
    val xSquared = d * d
    out[0] = atan(d[0])

    if (d.size > 1) {
        out[1] = d[1] / (1 + xSquared[0])

        for (n in 2 until d.size) {
            var sum = 0.0
            for (k in 1..n) {
                sum += bellPolynomial(n, k, xSquared) * d[k]
            }
            out[n] = -sum / (1 + xSquared[0])
        }
    }

    return DualNum(*out)
}


fun <Param> atan2(y: DualNum<Param>, x: DualNum<Param>): DualNum<Param> {
    val r2 = x * x + y * y  // r² = x² + y²
    val arctan = atan(y / x) // atan(y/x) computed element-wise

    val out = DoubleArray(max(y.size, x.size))
    out[0] = arctan[0] // atan2(y, x) at zeroth order

    if (out.size > 1) {
        out[1] = (x[0] * y[1] - y[0] * x[1]) / r2[0] // First derivative
    }

    if (out.size > 2) {
        for (n in 2 until out.size) {
            var sum = 0.0
            for (k in 1..n) {
                sum += bellPolynomial(n, k, r2) * (x * y[k] - y * x[k])[n - k]
            }
            out[n] = sum / r2[0]
        }
    }

    return DualNum(*out)
}


fun <Param>sqrt(d: DualNum<Param>): DualNum<Param> {
    if (d[0] < 0.0) throw ArithmeticException("Square root of negative number.")
    val out = DoubleArray(d.size)
    out[0] = kotlin.math.sqrt(d[0])
    if (d.size > 1) {
        out[1] = d[1] / (2 * out[0])
        for (n in 2 until d.size) {
            var sum = 0.0
            for (k in 1 until n) {
                sum += (k * out[k] * out[n - k])
            }
            out[n] = (d[n] - sum) / (2 * out[0])
        }
    }
    return DualNum(*out)
}
