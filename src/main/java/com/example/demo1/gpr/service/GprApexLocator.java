package com.example.demo1.gpr.service;

import java.util.Arrays;
import java.util.List;

import org.jtransforms.fft.DoubleFFT_1D;

import com.example.demo1.gpr.model.GprDetection;

/**
 * 在 YOLO 框内用 A-scan 包络寻找双曲线顶部。这里用局部抛物线近似波形脊线，
 * 只输出矩阵中的反射顶点，不把它解释为已校准的管道中心或埋深。
 */
final class GprApexLocator {

    // 人工框选时顶点位于框的上半部；留出顶部 5% 避开框沿和浅层水平反射。
    private static final double APEX_LEFT_FRACTION = 0.35;
    private static final double APEX_RIGHT_FRACTION = 0.65;
    private static final double APEX_TOP_FRACTION = 0.10;
    private static final double APEX_BOTTOM_FRACTION = 0.40;

    private GprApexLocator() {
    }

    static void locateAll(double[][] rawData, List<GprDetection> detections) {
        if (rawData == null || rawData.length < 8 || rawData[0].length < 12
                || detections == null || detections.isEmpty()) {
            return;
        }

        EnvelopeCache cache = new EnvelopeCache(rawData);
        for (GprDetection detection : detections) {
            Apex apex = locate(cache, detection);
            if (apex != null) {
                detection.setApexCol(apex.col());
                detection.setApexRow(apex.row());
                detection.setApexScore(apex.score());
                detection.setApexMethod(apex.method());
            }
        }
    }

    static Apex locate(double[][] rawData, GprDetection detection) {
        if (rawData == null || rawData.length < 8 || rawData[0].length < 12) {
            return null;
        }
        return locate(new EnvelopeCache(rawData), detection);
    }

    private static Apex locate(EnvelopeCache cache, GprDetection detection) {
        if (detection == null)
            return null;
        int rows = cache.rawData.length;
        int cols = cache.rawData[0].length;
        double boxWidth = detection.getX2() - detection.getX1();
        double boxHeight = detection.getY2() - detection.getY1();
        if (!Double.isFinite(boxWidth) || !Double.isFinite(boxHeight)
                || boxWidth < 12 || boxHeight < 8)
            return null;

        int x1 = clamp((int) Math.floor(detection.getX1() - boxWidth * 0.04), 0, cols - 1);
        int x2 = clamp((int) Math.ceil(detection.getX2() + boxWidth * 0.04), 0, cols - 1);
        int y1 = clamp((int) Math.floor(detection.getY1() - boxHeight * 0.05), 0, rows - 1);
        int y2 = clamp((int) Math.ceil(detection.getY2() + boxHeight * 0.05), 0, rows - 1);
        int width = x2 - x1 + 1;
        int height = y2 - y1 + 1;
        if (width < 12 || height < 8)
            return null;

        // ROI 可略微外扩以读取两翼，但顶点必须落在原 YOLO 方框的规定区域。
        int minApexX = clamp((int) Math.ceil(detection.getX1() + boxWidth * APEX_LEFT_FRACTION), x1, x2);
        int maxApexX = clamp((int) Math.floor(detection.getX1() + boxWidth * APEX_RIGHT_FRACTION), x1, x2);
        int minApexY = clamp((int) Math.ceil(detection.getY1() + boxHeight * APEX_TOP_FRACTION), y1, y2);
        int maxApexY = clamp((int) Math.floor(detection.getY1() + boxHeight * APEX_BOTTOM_FRACTION), y1, y2);
        if (minApexX > maxApexX || minApexY > maxApexY)
            return null;
        double boxCenterX = (detection.getX1() + detection.getX2()) / 2.0;
        double boxHalfWidth = boxWidth / 2.0;

        cache.ensureColumns(x1, x2);
        double[][] contrast = new double[height][width];
        double[] rowValues = new double[width];
        double[] positiveValues = new double[height * width];
        int positiveCount = 0;
        for (int row = 0; row < height; row++) {
            for (int col = 0; col < width; col++) {
                rowValues[col] = cache.envelope[y1 + row][x1 + col];
            }
            Arrays.sort(rowValues);
            double background = rowValues[width / 2];
            for (int col = 0; col < width; col++) {
                double value = cache.envelope[y1 + row][x1 + col] - background;
                contrast[row][col] = value;
                if (value > 0)
                    positiveValues[positiveCount++] = value;
            }
        }
        if (positiveCount < width)
            return null;
        Arrays.sort(positiveValues, 0, positiveCount);
        double scale = positiveValues[(int) Math.floor((positiveCount - 1) * 0.85)];
        if (!(scale > 1e-9))
            return null;

        int samples = Math.min(36, Math.max(16, width / 3));
        int peakRadius = clamp((int) Math.round(height * 0.02), 1, 3);
        int xStep = Math.max(1, width / 36);
        int yStep = Math.max(1, height / 28);
        double riseStep = height * 0.09;
        double halfWidth = Math.max(1.0, (width - 1) / 2.0);

        // 双翼仍然优先；仅在没有可靠双翼候选时，用连续的单翼轨迹推定顶点。
        Candidate[] best = new Candidate[2];
        for (int apexX = minApexX; apexX <= maxApexX; apexX += xStep) {
            for (int apexY = minApexY; apexY <= maxApexY; apexY += yStep) {
                for (double rise = height * 0.18; rise <= height * 1.09; rise += riseStep) {
                    Candidate candidate = evaluate(contrast, x1, y1, apexX, apexY, rise,
                            halfWidth, samples, peakRadius, scale, boxCenterX, boxHalfWidth);
                    consider(candidate, best);
                }
            }
        }
        for (Candidate coarse : best.clone()) {
            if (coarse == null)
                continue;
            for (int apexX = Math.max(minApexX, coarse.col() - xStep); apexX <= Math.min(maxApexX,
                    coarse.col() + xStep); apexX++) {
                for (int apexY = Math.max(minApexY, coarse.row() - yStep); apexY <= Math.min(maxApexY,
                        coarse.row() + yStep); apexY++) {
                    for (double rise = Math.max(height * 0.18, coarse.rise() - riseStep); rise <= Math
                            .min(height * 1.09, coarse.rise() + riseStep); rise += Math.max(1.0, riseStep / 4.0)) {
                        Candidate candidate = evaluate(contrast, x1, y1, apexX, apexY, rise,
                                halfWidth, samples, peakRadius, scale, boxCenterX, boxHalfWidth);
                        consider(candidate, best);
                    }
                }
            }
        }

        if (best[0] != null) {
            return new Apex(best[0].col(), best[0].row(),
                    Math.min(1.0, best[0].score() / 1.5), "bilateral");
        }
        if (best[1] != null) {
            double[] fitted = fitSingleWingApex(contrast, x1, y1, best[1], halfWidth,
                    peakRadius, scale, boxWidth, minApexX, maxApexX, minApexY, maxApexY);
            return new Apex(fitted[0], fitted[1],
                    Math.min(1.0, best[1].singleScore() / 1.5), "singleWing");
        }
        return null;
    }

    private static void consider(Candidate candidate, Candidate[] best) {
        if (candidate == null)
            return;
        if (candidate.score() >= 0.45 && candidate.minArm() >= 0.30
                && candidate.bilateralTrace()
                && candidate.apexStrength() >= 0.25
                && (best[0] == null || candidate.ranking() > best[0].ranking())) {
            best[0] = candidate;
        }
        if (candidate.singleScore() > 0
                && (best[1] == null || candidate.singleRanking() > best[1].singleRanking())) {
            best[1] = candidate;
        }
    }

    private static Candidate evaluate(double[][] contrast, int x1, int y1,
            int apexX, int apexY, double rise, double halfWidth,
            int samples, int peakRadius, double scale,
            double boxCenterX, double boxHalfWidth) {
        int width = contrast[0].length;
        int height = contrast.length;
        double left = 0;
        double right = 0;
        int leftCount = 0;
        int rightCount = 0;
        double apexStrength = localAverage(contrast, apexY - y1, apexX - x1, peakRadius) / scale;

        for (int i = 0; i < samples; i++) {
            double x = (width - 1) * (0.04 + 0.92 * i / (samples - 1.0));
            double displacement = (x1 + x - apexX) / halfWidth;
            int y = (int) Math.round(apexY - y1 + rise * displacement * displacement);
            if (y < 0 || y >= height)
                continue;
            int col = clamp((int) Math.round(x), 0, width - 1);
            double value = localAverage(contrast, y, col, peakRadius) / scale;
            if (x1 + x < apexX) {
                left += value;
                leftCount++;
            } else {
                right += value;
                rightCount++;
            }
        }
        if (leftCount < samples * 0.17 || rightCount < samples * 0.17
                || leftCount + rightCount < samples * 0.72)
            return null;
        double leftMean = left / leftCount;
        double rightMean = right / rightCount;
        double minArm = Math.min(leftMean, rightMean);
        double mean = (left + right) / (leftCount + rightCount);
        double score = 0.65 * mean + 0.25 * minArm + 0.10 * apexStrength;
        double centerOffset = (apexX - boxCenterX) / boxHalfWidth;
        double ranking = score - 0.25 * centerOffset * centerOffset;
        ArmTrace leftTrace = traceArm(contrast, x1, y1, apexX, apexY, rise,
                halfWidth, peakRadius, scale, -1, boxHalfWidth * 2);
        ArmTrace rightTrace = traceArm(contrast, x1, y1, apexX, apexY, rise,
                halfWidth, peakRadius, scale, 1, boxHalfWidth * 2);
        double singleScore = Math.max(leftTrace.score(), rightTrace.score());
        // 有清楚的一翼并且轨迹接近顶端，另一翼可以弱或缺失。
        if (singleScore < 0.45 || apexStrength < 0.10)
            singleScore = 0;
        double singleRanking = singleScore + 0.08 * Math.min(apexStrength, 2.0)
                - 3.0 * centerOffset * centerOffset;
        return new Candidate(apexX, apexY, rise, score, minArm, apexStrength,
                ranking, singleScore, singleRanking,
                leftTrace.score() >= rightTrace.score() ? -1 : 1,
                leftTrace.score() >= 0.40 && rightTrace.score() >= 0.40);
    }

    private static double[] fitSingleWingApex(double[][] contrast, int x1, int y1,
            Candidate candidate, double halfWidth,
            int peakRadius, double scale, double boxWidth,
            int minApexX, int maxApexX,
            int minApexY, int maxApexY) {
        double[][] normal = new double[3][4];
        int points = 0;
        int searchRadius = Math.max(3, peakRadius + 2);
        int step = Math.max(2, (int) Math.round(boxWidth / 45.0));
        for (int distance = step * 2; distance <= boxWidth * 0.57; distance += step) {
            int x = candidate.col() + candidate.singleDirection() * distance;
            int col = x - x1;
            if (col < 0 || col >= contrast[0].length)
                break;
            double displacement = (double) distance / halfWidth;
            int expectedRow = (int) Math.round(candidate.row() - y1
                    + candidate.rise() * displacement * displacement);
            int bestRow = -1;
            double bestStrength = 0.28;
            for (int row = Math.max(0, expectedRow - searchRadius); row <= Math.min(contrast.length - 1,
                    expectedRow + searchRadius); row++) {
                double strength = localAverage(contrast, row, col, peakRadius) / scale;
                if (strength > bestStrength) {
                    bestStrength = strength;
                    bestRow = row;
                }
            }
            if (bestRow < 0)
                continue;
            double dx = x - candidate.col();
            double[] basis = { 1, dx, dx * dx };
            double dy = y1 + bestRow - candidate.row();
            double weight = Math.min(2.0, bestStrength);
            for (int i = 0; i < 3; i++) {
                normal[i][3] += weight * basis[i] * dy;
                for (int j = 0; j < 3; j++) {
                    normal[i][j] += weight * basis[i] * basis[j];
                }
            }
            points++;
        }
        if (points < 6)
            return new double[] { candidate.col(), candidate.row() };
        for (int pivot = 0; pivot < 3; pivot++) {
            int best = pivot;
            for (int row = pivot + 1; row < 3; row++) {
                if (Math.abs(normal[row][pivot]) > Math.abs(normal[best][pivot]))
                    best = row;
            }
            if (Math.abs(normal[best][pivot]) < 1e-8) {
                return new double[] { candidate.col(), candidate.row() };
            }
            double[] swap = normal[pivot];
            normal[pivot] = normal[best];
            normal[best] = swap;
            double diagonal = normal[pivot][pivot];
            for (int col = pivot; col < 4; col++)
                normal[pivot][col] /= diagonal;
            for (int row = 0; row < 3; row++) {
                if (row == pivot)
                    continue;
                double factor = normal[row][pivot];
                for (int col = pivot; col < 4; col++) {
                    normal[row][col] -= factor * normal[pivot][col];
                }
            }
        }
        double intercept = normal[0][3];
        double slope = normal[1][3];
        double curvature = normal[2][3];
        if (!(curvature > 0))
            return new double[] { candidate.col(), candidate.row() };
        double fittedX = candidate.col() - slope / (2 * curvature);
        double fittedY = candidate.row() + intercept - slope * slope / (4 * curvature);
        if (!Double.isFinite(fittedX) || !Double.isFinite(fittedY)
                || Math.abs(fittedX - candidate.col()) > boxWidth * 0.18
                || fittedX < minApexX || fittedX > maxApexX
                || fittedY < minApexY || fittedY > maxApexY) {
            return new double[] { candidate.col(), candidate.row() };
        }
        return new double[] { fittedX, fittedY };
    }

    private static ArmTrace traceArm(double[][] contrast, int x1, int y1,
            int apexX, int apexY, double rise, double halfWidth,
            int peakRadius, double scale, int direction,
            double boxWidth) {
        int width = contrast[0].length;
        int height = contrast.length;
        int edge = direction < 0 ? x1 : x1 + width - 1;
        int available = Math.abs(edge - apexX);
        if (available < boxWidth * 0.22)
            return new ArmTrace(0);

        // 沿候选抛物线从顶端向外采样。只允许短暂中断，不能把离散亮点拼成一翼。
        int step = Math.max(1, available / 18);
        int firstSupported = -1;
        int lastSupported = -1;
        int supported = 0;
        int gaps = 0;
        int longestGap = 0;
        double sum = 0;
        int count = 0;
        for (int distance = step; distance <= available; distance += step) {
            int col = apexX + direction * distance - x1;
            double displacement = (double) distance / halfWidth;
            int row = (int) Math.round(apexY - y1 + rise * displacement * displacement);
            if (row < 0 || row >= height || col < 0 || col >= width)
                break;
            double strength = localAverage(contrast, row, col, peakRadius) / scale;
            count++;
            if (strength >= 0.28) {
                if (firstSupported < 0)
                    firstSupported = distance;
                lastSupported = distance;
                supported++;
                sum += strength;
                if (firstSupported >= 0)
                    longestGap = Math.max(longestGap, gaps);
                gaps = 0;
            } else if (firstSupported >= 0) {
                gaps++;
            }
        }
        if (firstSupported < 0 || firstSupported > boxWidth * 0.12
                || lastSupported - firstSupported < boxWidth * 0.20
                || supported < 6 || count < 8 || longestGap > 2)
            return new ArmTrace(0);
        double depthChange = rise * (lastSupported * lastSupported
                - firstSupported * firstSupported) / (halfWidth * halfWidth);
        if (depthChange < Math.max(3.0, height * 0.07))
            return new ArmTrace(0);
        double coverage = (double) supported / count;
        if (coverage < 0.57)
            return new ArmTrace(0);
        return new ArmTrace(sum / supported * coverage);
    }

    private static double localAverage(double[][] contrast, int row, int col, int radius) {
        double weightedSum = 0;
        double weightTotal = 0;
        for (int r = Math.max(0, row - radius); r <= Math.min(contrast.length - 1, row + radius); r++) {
            double weight = radius + 1 - Math.abs(r - row);
            weightedSum += weight * contrast[r][col];
            weightTotal += weight;
        }
        return weightedSum / weightTotal;
    }

    private static int clamp(int value, int min, int max) {
        return Math.max(min, Math.min(max, value));
    }

    record Apex(double col, double row, double score, String method) {
    }

    private record Candidate(int col, int row, double rise, double score,
            double minArm, double apexStrength, double ranking,
            double singleScore, double singleRanking,
            int singleDirection, boolean bilateralTrace) {
    }

    private record ArmTrace(double score) {
    }

    private static final class EnvelopeCache {
        private final double[][] rawData;
        private final double[][] envelope;
        private final boolean[] computed;
        private final DoubleFFT_1D fft;

        private EnvelopeCache(double[][] rawData) {
            this.rawData = rawData;
            this.envelope = new double[rawData.length][rawData[0].length];
            this.computed = new boolean[rawData[0].length];
            this.fft = new DoubleFFT_1D(rawData.length);
        }

        private void ensureColumns(int x1, int x2) {
            int rows = rawData.length;
            double[] spectrum = new double[2 * rows];
            for (int col = x1; col <= x2; col++) {
                if (computed[col])
                    continue;
                for (int row = 0; row < rows; row++) {
                    spectrum[2 * row] = Double.isFinite(rawData[row][col]) ? rawData[row][col] : 0;
                    spectrum[2 * row + 1] = 0;
                }
                fft.complexForward(spectrum);
                for (int frequency = 1; frequency < rows; frequency++) {
                    if (frequency < (rows + 1) / 2) {
                        spectrum[2 * frequency] *= 2;
                        spectrum[2 * frequency + 1] *= 2;
                    } else if (frequency > rows / 2 || rows % 2 != 0) {
                        spectrum[2 * frequency] = 0;
                        spectrum[2 * frequency + 1] = 0;
                    }
                }
                fft.complexInverse(spectrum, true);
                for (int row = 0; row < rows; row++) {
                    envelope[row][col] = Math.hypot(spectrum[2 * row], spectrum[2 * row + 1]);
                }
                computed[col] = true;
            }
        }
    }
}
