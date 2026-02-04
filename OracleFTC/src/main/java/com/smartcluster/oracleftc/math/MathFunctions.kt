package com.smartcluster.oracleftc.math

private val bellCache = Array(100) { Array(100) { DoubleArray(100) { 0.0 } } }
fun <P> bellPolynomial(n: Int, k: Int, dual: DualNum<P>): Double {
    if (n == 0 && k == 0) return 1.0
    if (k == 0 || n < k) return 0.0

    if (bellCache[n][k][0] != 0.0) return bellCache[n][k][0]

    var sum = 0.0
    for (m in 1 until n - k + 2) {
        sum += choose(n - 1, m - 1) * dual[m] * bellPolynomial(n - m, k - 1, dual)
    }
    bellCache[n][k][0] = sum
    return sum
}

private val chooseCache = Array(100) { DoubleArray(100) { -1.0 } }
fun choose(n: Int, k: Int): Double {
    if (k > n || k < 0) return 0.0
    if (k == 0 || k == n) return 1.0
    if (chooseCache[n][k] != -1.0) return chooseCache[n][k]
    chooseCache[n][k] = choose(n - 1, k - 1) + choose(n - 1, k)
    return chooseCache[n][k]
}