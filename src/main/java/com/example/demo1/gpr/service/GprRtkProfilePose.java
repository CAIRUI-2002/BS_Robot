package com.example.demo1.gpr.service;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

/**
 * 用一组有噪声的RTK逐道坐标估计整张GPR剖面的刚体位姿。
 *
 * <p>RTK只决定剖面的中心和方向；剖面内部各道仍严格使用DZT道间距，
 * 不允许单个RTK点把二维剖面拉弯或压缩。</p>
 */
final class GprRtkProfilePose {

    private static final double MIN_OUTLIER_TOLERANCE_M = 0.05;

    private GprRtkProfilePose() {
    }

    static RigidPath fit(double[] sampleX, double[] sampleY, int traceCount, double traceSpacingM) {
        if (sampleX == null || sampleY == null || sampleX.length != sampleY.length
                || sampleX.length < 2 || traceCount < 2
                || !Double.isFinite(traceSpacingM) || traceSpacingM <= 0) {
            return null;
        }

        List<Sample> samples = new ArrayList<>();
        for (int i = 0; i < sampleX.length; i++) {
            if (Double.isFinite(sampleX[i]) && Double.isFinite(sampleY[i])) {
                samples.add(new Sample(i, sampleX[i], sampleY[i]));
            }
        }
        if (samples.size() < 2) return null;

        LineFit initial = fitLine(samples);
        if (initial == null) return null;

        double[] residuals = samples.stream()
                .mapToDouble(sample -> perpendicularResidual(sample, initial))
                .toArray();
        double residualMedian = median(residuals);
        double[] deviations = Arrays.stream(residuals)
                .map(value -> Math.abs(value - residualMedian))
                .toArray();
        double mad = median(deviations);
        double threshold = residualMedian + Math.max(
                MIN_OUTLIER_TOLERANCE_M,
                4.0 * 1.4826 * mad
        );

        List<Sample> inliers = new ArrayList<>();
        for (int i = 0; i < samples.size(); i++) {
            if (residuals[i] <= threshold) inliers.add(samples.get(i));
        }
        if (inliers.size() < Math.max(2, (int) Math.ceil(samples.size() * 0.5))) {
            inliers = samples;
        }

        LineFit refined = fitLine(inliers);
        if (refined == null) return null;

        double directionX = refined.directionX;
        double directionY = refined.directionY;
        double[] orderedDirection = estimateOrderedDirection(samples);
        if (directionX * orderedDirection[0] + directionY * orderedDirection[1] < 0) {
            directionX = -directionX;
            directionY = -directionY;
        }

        double length = (traceCount - 1) * traceSpacingM;
        double[] traceX = new double[traceCount];
        double[] traceY = new double[traceCount];
        double centerColumn = (traceCount - 1) / 2.0;
        for (int c = 0; c < traceCount; c++) {
            double along = (c - centerColumn) * traceSpacingM;
            traceX[c] = refined.centerX + along * directionX;
            traceY[c] = refined.centerY + along * directionY;
        }

        double squaredResidualSum = 0;
        for (Sample sample : inliers) {
            double residual = perpendicularResidual(
                    sample,
                    new LineFit(refined.centerX, refined.centerY, directionX, directionY)
            );
            squaredResidualSum += residual * residual;
        }
        double rmse = Math.sqrt(squaredResidualSum / inliers.size());

        return new RigidPath(
                traceX,
                traceY,
                Math.atan2(directionY, directionX),
                length,
                inliers.size(),
                samples.size(),
                rmse
        );
    }

    private static LineFit fitLine(List<Sample> samples) {
        double centerX = samples.stream().mapToDouble(Sample::x).average().orElse(Double.NaN);
        double centerY = samples.stream().mapToDouble(Sample::y).average().orElse(Double.NaN);
        if (!Double.isFinite(centerX) || !Double.isFinite(centerY)) return null;

        double covarianceXX = 0;
        double covarianceXY = 0;
        double covarianceYY = 0;
        for (Sample sample : samples) {
            double dx = sample.x - centerX;
            double dy = sample.y - centerY;
            covarianceXX += dx * dx;
            covarianceXY += dx * dy;
            covarianceYY += dy * dy;
        }
        if (covarianceXX + covarianceYY <= 1e-12) return null;

        double angle = 0.5 * Math.atan2(2.0 * covarianceXY, covarianceXX - covarianceYY);
        return new LineFit(centerX, centerY, Math.cos(angle), Math.sin(angle));
    }

    private static double perpendicularResidual(Sample sample, LineFit fit) {
        double dx = sample.x - fit.centerX;
        double dy = sample.y - fit.centerY;
        return Math.abs(-fit.directionY * dx + fit.directionX * dy);
    }

    private static double[] estimateOrderedDirection(List<Sample> samples) {
        int window = Math.max(1, Math.min(10, samples.size() / 10));
        if (window == 0) window = 1;

        double[] firstX = new double[window];
        double[] firstY = new double[window];
        double[] lastX = new double[window];
        double[] lastY = new double[window];
        for (int i = 0; i < window; i++) {
            Sample first = samples.get(i);
            Sample last = samples.get(samples.size() - window + i);
            firstX[i] = first.x;
            firstY[i] = first.y;
            lastX[i] = last.x;
            lastY[i] = last.y;
        }
        return new double[]{median(lastX) - median(firstX), median(lastY) - median(firstY)};
    }

    private static double median(double[] values) {
        double[] sorted = values.clone();
        Arrays.sort(sorted);
        int middle = sorted.length / 2;
        return sorted.length % 2 == 0
                ? (sorted[middle - 1] + sorted[middle]) / 2.0
                : sorted[middle];
    }

    record RigidPath(
            double[] traceX,
            double[] traceY,
            double phiRad,
            double length,
            int inlierCount,
            int sampleCount,
            double rmseM
    ) {
    }

    private record Sample(int index, double x, double y) {
    }

    private record LineFit(double centerX, double centerY, double directionX, double directionY) {
    }
}
