package com.example.demo1.gpr.service;

import org.jtransforms.fft.DoubleFFT_2D;

/**
 * 基于 JTransforms 的 FFT 工具类：
 * - 简单复数类型 Complex
 * - 2D FFT / IFFT
 * - 沿 x 方向的 fftshift / ifftshift
 * - 一维复数线性插值（用于 Stolt 映射）
 */
public class FftUtils {

    /**
     * 简单复数类型
     */
    public static class Complex {
        public final double re;
        public final double im;

        public Complex(double re, double im) {
            this.re = re;
            this.im = im;
        }

        public Complex add(Complex other) {
            return new Complex(this.re + other.re, this.im + other.im);
        }

        public Complex scale(double k) {
            return new Complex(this.re * k, this.im * k);
        }

        public Complex conj() {
            return new Complex(this.re, -this.im);
        }

        public static Complex zero() {
            return new Complex(0.0, 0.0);
        }
    }

    // ========================= 2D FFT / IFFT =============================

    /**
     * 对 Nt x Nx 的复数矩阵做 2D FFT（就地不改输入，返回新矩阵）
     */
    public static Complex[][] fft2D(Complex[][] data) {
        int rows = data.length;
        int cols = data[0].length;

        double[] arr = new double[2 * rows * cols];
        // 填充实部/虚部
        for (int r = 0; r < rows; r++) {
            for (int c = 0; c < cols; c++) {
                int idx = 2 * (r * cols + c);
                arr[idx] = data[r][c].re;
                arr[idx + 1] = data[r][c].im;
            }
        }

        DoubleFFT_2D fft2D = new DoubleFFT_2D(rows, cols);
        fft2D.complexForward(arr); // 不缩放

        Complex[][] out = new Complex[rows][cols];
        for (int r = 0; r < rows; r++) {
            for (int c = 0; c < cols; c++) {
                int idx = 2 * (r * cols + c);
                out[r][c] = new Complex(arr[idx], arr[idx + 1]);
            }
        }
        return out;
    }

    /**
     * 对 Nt x Nx 的复数矩阵做 2D IFFT，返回 Complex[][]。
     * 使用 scale=true，使得结果自动除以 (rows*cols)。
     */
    public static Complex[][] ifft2D(Complex[][] data) {
        int rows = data.length;
        int cols = data[0].length;

        double[] arr = new double[2 * rows * cols];
        for (int r = 0; r < rows; r++) {
            for (int c = 0; c < cols; c++) {
                int idx = 2 * (r * cols + c);
                arr[idx] = data[r][c].re;
                arr[idx + 1] = data[r][c].im;
            }
        }

        DoubleFFT_2D fft2D = new DoubleFFT_2D(rows, cols);
        fft2D.complexInverse(arr, true); // scale=true

        Complex[][] out = new Complex[rows][cols];
        for (int r = 0; r < rows; r++) {
            for (int c = 0; c < cols; c++) {
                int idx = 2 * (r * cols + c);
                out[r][c] = new Complex(arr[idx], arr[idx + 1]);
            }
        }
        return out;
    }

    // ========================= fftshift / ifftshift （按列） =============================

    /**
     * 沿 x 方向（列）做 fftshift，相当于 MATLAB 的 fftshift(A,2)
     */
    public static Complex[][] fftshiftX(Complex[][] in) {
        int rows = in.length;
        int cols = in[0].length;
        int half = cols / 2;

        Complex[][] out = new Complex[rows][cols];
        for (int r = 0; r < rows; r++) {
            for (int c = 0; c < cols; c++) {
                int newC = (c + half) % cols;
                out[r][newC] = in[r][c];
            }
        }
        return out;
    }

    /**
     * 沿 x 方向（列）做 ifftshift，相当于 MATLAB 的 ifftshift(A,2)
     */
    public static Complex[][] ifftshiftX(Complex[][] in) {
        int rows = in.length;
        int cols = in[0].length;
        int half = cols / 2;

        Complex[][] out = new Complex[rows][cols];
        for (int r = 0; r < rows; r++) {
            for (int c = 0; c < cols; c++) {
                int newC = (c + cols - half) % cols;
                out[r][newC] = in[r][c];
            }
        }
        return out;
    }

    // ========================= 一维复数线性插值 =============================

    /**
     * 对一个复数列 y(w) 在新频率 ws 上做线性插值。
     * x: 原自变量（递增），长度 N
     * y: 原复数值，长度 N
     * xq: 新自变量数组，长度 M
     * 返回长度 M 的 Complex[]
     * 超出 [x[0], x[N-1]] 的点，返回 0+0i。
     */
    public static Complex[] interp1Complex(double[] x, Complex[] y, double[] xq) {
        int n = x.length;
        Complex[] out = new Complex[xq.length];

        for (int i = 0; i < xq.length; i++) {
            double v = xq[i];

            if (v < x[0] || v > x[n - 1]) {
                out[i] = Complex.zero();
                continue;
            }

            int j = findLeIndex(x, v);
            if (j < 0 || j >= n - 1) {
                out[i] = Complex.zero();
                continue;
            }

            double x1 = x[j];
            double x2 = x[j + 1];
            double t = (v - x1) / (x2 - x1);

            double re = y[j].re * (1 - t) + y[j + 1].re * t;
            double im = y[j].im * (1 - t) + y[j + 1].im * t;

            out[i] = new Complex(re, im);
        }

        return out;
    }

    /**
     * 在有序数组 x 中找到最大的 j，使得 x[j] <= target 且 j+1 仍在范围内。
     * 若找不到则返回 -1。
     */
    private static int findLeIndex(double[] x, double target) {
        int n = x.length;
        if (target < x[0] || target > x[n - 1]) {
            return -1;
        }
        int lo = 0;
        int hi = n - 1;
        while (hi - lo > 1) {
            int mid = (lo + hi) / 2;
            if (x[mid] <= target) {
                lo = mid;
            } else {
                hi = mid;
            }
        }
        return lo;
    }
}
