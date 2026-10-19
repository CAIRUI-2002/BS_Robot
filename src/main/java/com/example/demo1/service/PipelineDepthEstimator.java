package com.example.demo1.service;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Comparator;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

/**
 * Pure Java implementation of the real-time cable-depth estimator.
 *
 * <p>The constants and processing order intentionally match the validated V2
 * algorithm: log calibration, time-aligned pose projection, signal-aware
 * uncertainty, a robust constant-slope Kalman filter and RTS smoothing.</p>
 */
final class PipelineDepthEstimator {

    private static final double DEPTH_MIN_M = 0.05;
    private static final double DEPTH_MAX_M = 5.0;
    private static final double DEEP_PRIOR_THRESHOLD_M = 2.5;
    private static final double SIGMA_SIGNAL_ZERO = 0.25;
    private static final double SIGMA_CROSSING = 0.40;
    private static final double SIGMA_SIDE = 0.65;
    private static final double SIGMA_UNKNOWN = 0.80;
    private static final double SLOPE_RANDOM_WALK_Q = 0.0025;
    private static final double INNOVATION_GATE_SIGMA = 3.0;
    private static final double MAX_R_INFLATION = 100.0;
    private static final double CROSSING_LOOKBACK_MS = 2500.0;
    private static final double SIGNAL_MATCH_MAX_GAP_MS = 500.0;
    private static final double HISTORY_KEEP_MS = 30000.0;
    private static final double NODE_SPACING_M = 0.10;

    // Validated calibration parameters formerly read from the Python model JSON files.
    private static final double SINGLE_INTERCEPT = -0.4616518371807156;
    private static final double SINGLE_SLOPE = 0.2998109811888858;
    private static final double WINDOW_INTERCEPT = -1.6914904031516205;
    private static final double WINDOW_SLOPE = 0.5232197266413372;

    private final List<TimedPose> poses = new ArrayList<>();
    private final List<TimedSignal> signals = new ArrayList<>();
    private final List<TimedPoint> cablePoints = new ArrayList<>();
    private final List<Double> rawDepthHistory = new ArrayList<>();
    private final List<DepthObservation> observations = new ArrayList<>();
    private final List<DepthEvent> pendingDepths = new ArrayList<>();

    private FilterResult lastResult;
    private List<Point2> lastNodesXY = List.of();
    private double[] lastNodesS = new double[0];
    private boolean finalized;

    void addPose(double timestampMs, double xM, double yM) {
        poses.add(new TimedPose(timestampMs, xM, yM));
        orderAndTrim(poses);
    }

    void addSignal(double timestampMs, int signal) {
        if (signal < 0 || signal > 2) return;
        signals.add(new TimedSignal(timestampMs, signal));
        orderAndTrim(signals);
    }

    void addCablePoint(double timestampMs, double xM, double yM) {
        if (!cablePoints.isEmpty()) {
            TimedPoint previous = cablePoints.get(cablePoints.size() - 1);
            if (Math.hypot(xM - previous.x(), yM - previous.y()) < 1e-6) return;
        }
        cablePoints.add(new TimedPoint(timestampMs, xM, yM));
        processPending(false);
        recompute();
    }

    void replaceCablePath(double timestampMs, List<Point2> points) {
        cablePoints.clear();
        int offset = 0;
        for (Point2 point : points) {
            if (point == null || !Double.isFinite(point.x()) || !Double.isFinite(point.y())) continue;
            if (!cablePoints.isEmpty()) {
                TimedPoint previous = cablePoints.get(cablePoints.size() - 1);
                if (Math.hypot(point.x() - previous.x(), point.y() - previous.y()) < 1e-6) continue;
            }
            cablePoints.add(new TimedPoint(timestampMs + offset * 0.001, point.x(), point.y()));
            offset++;
        }
        processPending(false);
        recompute();
    }

    DepthObservation addDepth(
            double receiptTimeMs,
            double rawDepthCm,
            Double captureTimeMs,
            Double backendReceiveTimeMs
    ) {
        DepthEvent event = new DepthEvent(receiptTimeMs, rawDepthCm, captureTimeMs, backendReceiveTimeMs);
        DepthObservation observation = makeObservation(event);
        if (observation == null) {
            pendingDepths.add(event);
            return null;
        }
        observations.add(observation);
        recompute();
        return observation;
    }

    List<Map<String, Object>> getMap() {
        if (lastResult == null) return List.of();
        List<Map<String, Object>> rows = new ArrayList<>(lastNodesS.length);
        for (int index = 0; index < lastNodesS.length; index++) {
            Point2 point = lastNodesXY.get(index);
            double depth = lastResult.finalDepth()[index];
            double sigma = lastResult.finalSigma()[index];
            String source = lastResult.sourceCode()[index];
            Map<String, Object> row = new LinkedHashMap<>();
            row.put("node_index", index);
            row.put("s_m", lastNodesS[index]);
            row.put("x_m", point.x());
            row.put("y_m", point.y());
            row.put("depth_m", Double.isFinite(depth) ? depth : null);
            row.put("uncertainty_1sigma_m", Double.isFinite(sigma) ? sigma : null);
            row.put("confidence", Double.isFinite(sigma) ? confidenceLabel(sigma, source) : "unknown");
            row.put("source", source);
            row.put("measurement_count_at_node", lastResult.measurementCount()[index]);
            row.put("depth_zone", Double.isFinite(depth) && depth <= DEEP_PRIOR_THRESHOLD_M
                    ? "common_0_2.5m" : "allowed_deep_2.5_5m");
            row.put("finalized", finalized);
            rows.add(row);
        }
        return rows;
    }

    List<Map<String, Object>> finalizeMap() {
        processPending(true);
        recompute();
        finalized = true;
        return getMap();
    }

    int pendingCount() {
        return pendingDepths.size();
    }

    private <T extends TimedValue> void orderAndTrim(List<T> values) {
        if (values.size() >= 2
                && values.get(values.size() - 2).timestampMs() > values.get(values.size() - 1).timestampMs()) {
            values.sort(Comparator.comparingDouble(TimedValue::timestampMs));
        }
        double newest = values.get(values.size() - 1).timestampMs();
        while (!values.isEmpty() && newest - values.get(0).timestampMs() > HISTORY_KEEP_MS) {
            values.remove(0);
        }
    }

    private DepthObservation makeObservation(DepthEvent event) {
        double raw = event.rawDepthCm();
        if (!Double.isFinite(raw) || raw <= 0.0 || cablePoints.size() < 2) return null;
        double captureUnified = event.captureTimeMs() != null && event.backendReceiveTimeMs() != null
                ? event.receiptTimeMs() - (event.backendReceiveTimeMs() - event.captureTimeMs())
                : event.receiptTimeMs();
        Point2 pose = interpolatePose(captureUnified);
        if (pose == null) return null;

        List<Point2> polyline = cablePoints.stream().map(point -> new Point2(point.x(), point.y())).toList();
        Projection projection = projectPointToPolyline(pose, polyline);
        SignalContext context = classifySignalContext(captureUnified);
        Calibration calibration = calibrateWindow(raw, context.name(), captureUnified);
        boolean accepted = calibration.depthM() >= DEPTH_MIN_M && calibration.depthM() <= DEPTH_MAX_M;
        return new DepthObservation(
                "live",
                event.receiptTimeMs(),
                captureUnified,
                raw,
                calibration.depthM(),
                pose.x(),
                pose.y(),
                projection.point().x(),
                projection.point().y(),
                projection.sM(),
                projection.lateralM(),
                context.signal(),
                context.name(),
                calibration.sigmaM(),
                calibration.channel(),
                accepted,
                accepted && calibration.depthM() > DEEP_PRIOR_THRESHOLD_M
        );
    }

    private Point2 interpolatePose(double timestampMs) {
        if (poses.isEmpty()) return null;
        TimedPose first = poses.get(0);
        TimedPose last = poses.get(poses.size() - 1);
        if (timestampMs < first.timestampMs() - 500.0 || timestampMs > last.timestampMs() + 500.0) return null;
        if (timestampMs <= first.timestampMs()) return new Point2(first.x(), first.y());
        if (timestampMs >= last.timestampMs()) return new Point2(last.x(), last.y());
        int right = 1;
        while (right < poses.size() && poses.get(right).timestampMs() < timestampMs) right++;
        TimedPose before = poses.get(right - 1);
        TimedPose after = poses.get(right);
        double duration = after.timestampMs() - before.timestampMs();
        if (duration <= 0.0) return new Point2(after.x(), after.y());
        double ratio = (timestampMs - before.timestampMs()) / duration;
        return new Point2(
                before.x() + ratio * (after.x() - before.x()),
                before.y() + ratio * (after.y() - before.y())
        );
    }

    private Calibration calibrateWindow(double rawCm, String context, double captureTimeMs) {
        rawDepthHistory.add(rawCm);
        double calibrated = calibrateLog(rawCm, SINGLE_INTERCEPT, SINGLE_SLOPE);
        double baseSigma = classifySignalContext(captureTimeMs).sigmaM();
        if ("center_0".equals(context)) {
            return new Calibration(calibrated, SIGMA_SIGNAL_ZERO, "single_center_0");
        }
        if (rawDepthHistory.size() >= 4) {
            double[] window = new double[4];
            for (int i = 0; i < 4; i++) {
                window[i] = rawDepthHistory.get(rawDepthHistory.size() - 4 + i);
            }
            Arrays.sort(window);
            // NumPy's default q=0.25 linear quantile for four values: position 0.75.
            double q25 = window[0] * 0.25 + window[1] * 0.75;
            calibrated = calibrateLog(q25, WINDOW_INTERCEPT, WINDOW_SLOPE);
            return new Calibration(calibrated, SIGMA_CROSSING, "rolling_q25_4");
        }
        return new Calibration(calibrated, baseSigma, "single_fast");
    }

    private static double calibrateLog(double rawCm, double intercept, double slope) {
        return Math.max(0.0, intercept + slope * Math.log(rawCm));
    }

    private Integer signalAtTime(double timestampMs) {
        if (signals.isEmpty()) return null;
        int insertion = 0;
        while (insertion < signals.size() && signals.get(insertion).timestampMs() < timestampMs) insertion++;
        TimedSignal nearest = null;
        double nearestGap = Double.POSITIVE_INFINITY;
        if (insertion < signals.size()) {
            nearest = signals.get(insertion);
            nearestGap = Math.abs(nearest.timestampMs() - timestampMs);
        }
        if (insertion > 0) {
            TimedSignal left = signals.get(insertion - 1);
            double gap = Math.abs(left.timestampMs() - timestampMs);
            if (gap < nearestGap) {
                nearest = left;
                nearestGap = gap;
            }
        }
        return nearest != null && nearestGap <= SIGNAL_MATCH_MAX_GAP_MS ? nearest.signal() : null;
    }

    private SignalContext classifySignalContext(double timestampMs) {
        Integer signal = signalAtTime(timestampMs);
        if (signal != null && signal == 0) return new SignalContext("center_0", signal, SIGMA_SIGNAL_ZERO);
        List<Integer> compressed = new ArrayList<>();
        for (TimedSignal event : signals) {
            if (event.timestampMs() < timestampMs - CROSSING_LOOKBACK_MS || event.timestampMs() > timestampMs) continue;
            int value = event.signal();
            if ((compressed.isEmpty() || compressed.get(compressed.size() - 1) != value)) compressed.add(value);
        }
        if (compressed.contains(0) || (compressed.contains(1) && compressed.contains(2))) {
            return new SignalContext("crossing_recent", signal, SIGMA_CROSSING);
        }
        if (signal != null && (signal == 1 || signal == 2)) {
            return new SignalContext("side", signal, SIGMA_SIDE);
        }
        return new SignalContext("unknown", signal, SIGMA_UNKNOWN);
    }

    private void processPending(boolean force) {
        List<DepthEvent> remaining = new ArrayList<>();
        for (DepthEvent event : pendingDepths) {
            DepthObservation observation = makeObservation(event);
            if (observation == null) {
                if (!force) remaining.add(event);
            } else {
                observations.add(observation);
            }
        }
        pendingDepths.clear();
        pendingDepths.addAll(remaining);
    }

    private void recompute() {
        if (cablePoints.isEmpty()) {
            lastResult = null;
            lastNodesXY = List.of();
            lastNodesS = new double[0];
            return;
        }
        List<Point2> points = cablePoints.stream().map(point -> new Point2(point.x(), point.y())).toList();
        ResampledPath path = resamplePolyline(points, NODE_SPACING_M);
        lastNodesXY = path.points();
        lastNodesS = path.sValues();
        lastResult = runDepthFilter(lastNodesS, observations);
    }

    private static ResampledPath resamplePolyline(List<Point2> input, double spacing) {
        List<Point2> points = removeNearDuplicates(input);
        if (points.isEmpty()) return new ResampledPath(List.of(), new double[0]);
        if (points.size() == 1) return new ResampledPath(List.of(points.get(0)), new double[]{0.0});
        double[] originalS = cumulativeS(points);
        double total = originalS[originalS.length - 1];
        if (total <= 1e-9) return new ResampledPath(List.of(points.get(0)), new double[]{0.0});

        List<Double> samples = new ArrayList<>();
        for (int index = 0; index * spacing < total; index++) samples.add(index * spacing);
        if (samples.isEmpty() || Math.abs(samples.get(samples.size() - 1) - total) > 1e-9) samples.add(total);

        List<Point2> sampledPoints = new ArrayList<>(samples.size());
        double[] sampledS = new double[samples.size()];
        int segment = 0;
        for (int i = 0; i < samples.size(); i++) {
            double s = samples.get(i);
            sampledS[i] = s;
            while (segment + 1 < originalS.length - 1 && originalS[segment + 1] < s) segment++;
            double length = originalS[segment + 1] - originalS[segment];
            double ratio = length <= 0.0 ? 0.0 : (s - originalS[segment]) / length;
            Point2 start = points.get(segment);
            Point2 end = points.get(segment + 1);
            sampledPoints.add(new Point2(
                    start.x() + ratio * (end.x() - start.x()),
                    start.y() + ratio * (end.y() - start.y())
            ));
        }
        return new ResampledPath(sampledPoints, sampledS);
    }

    private static List<Point2> removeNearDuplicates(List<Point2> input) {
        if (input.size() <= 1) return new ArrayList<>(input);
        List<Point2> points = new ArrayList<>();
        Point2 last = input.get(0);
        points.add(last);
        for (int index = 1; index < input.size(); index++) {
            Point2 point = input.get(index);
            if (distance(point, last) > 1e-6) {
                points.add(point);
                last = point;
            }
        }
        return points;
    }

    private static double[] cumulativeS(List<Point2> points) {
        double[] values = new double[points.size()];
        for (int index = 1; index < points.size(); index++) {
            values[index] = values[index - 1] + distance(points.get(index - 1), points.get(index));
        }
        return values;
    }

    private static Projection projectPointToPolyline(Point2 point, List<Point2> points) {
        if (points.isEmpty()) return new Projection(Double.NaN, Double.POSITIVE_INFINITY, new Point2(Double.NaN, Double.NaN));
        if (points.size() == 1) return new Projection(0.0, distance(point, points.get(0)), points.get(0));
        double[] cumulative = cumulativeS(points);
        double bestDistanceSquared = Double.POSITIVE_INFINITY;
        double bestS = Double.NaN;
        Point2 bestPoint = null;
        for (int index = 0; index < points.size() - 1; index++) {
            Point2 start = points.get(index);
            Point2 end = points.get(index + 1);
            double dx = end.x() - start.x();
            double dy = end.y() - start.y();
            double lengthSquared = dx * dx + dy * dy;
            double ratio = lengthSquared > 1e-12
                    ? ((point.x() - start.x()) * dx + (point.y() - start.y()) * dy) / lengthSquared
                    : 0.0;
            ratio = clamp(ratio, 0.0, 1.0);
            Point2 projected = new Point2(start.x() + ratio * dx, start.y() + ratio * dy);
            double px = projected.x() - point.x();
            double py = projected.y() - point.y();
            double distanceSquared = px * px + py * py;
            if (distanceSquared < bestDistanceSquared) {
                bestDistanceSquared = distanceSquared;
                bestS = cumulative[index] + ratio * Math.sqrt(lengthSquared);
                bestPoint = projected;
            }
        }
        return new Projection(bestS, Math.sqrt(bestDistanceSquared), bestPoint);
    }

    private static FilterResult runDepthFilter(double[] nodesS, List<DepthObservation> observations) {
        int count = nodesS.length;
        if (count == 0) return new FilterResult(new double[0], new double[0], new int[0], new String[0]);
        List<List<DepthObservation>> byNode = new ArrayList<>(count);
        for (int index = 0; index < count; index++) byNode.add(new ArrayList<>());
        for (DepthObservation observation : observations) {
            if (!observation.acceptedPhysicalRange()) continue;
            int nearest = 0;
            double nearestDistance = Math.abs(nodesS[0] - observation.sM());
            for (int index = 1; index < count; index++) {
                double candidate = Math.abs(nodesS[index] - observation.sM());
                if (candidate < nearestDistance) {
                    nearest = index;
                    nearestDistance = candidate;
                }
            }
            byNode.get(nearest).add(observation);
        }
        int[] measurementCount = byNode.stream().mapToInt(List::size).toArray();
        int firstIndex = -1;
        for (int index = 0; index < count; index++) {
            if (!byNode.get(index).isEmpty()) {
                firstIndex = index;
                break;
            }
        }
        if (firstIndex < 0) {
            double[] missingDepth = new double[count];
            double[] missingSigma = new double[count];
            Arrays.fill(missingDepth, Double.NaN);
            Arrays.fill(missingSigma, Double.NaN);
            String[] sources = new String[count];
            Arrays.fill(sources, "unknown");
            return new FilterResult(missingDepth, missingSigma, measurementCount, sources);
        }

        DepthObservation first = byNode.get(firstIndex).get(0);
        double[] state = {clamp(first.calibratedDepthM(), DEPTH_MIN_M, DEPTH_MAX_M), 0.0};
        double[][] covariance = {{Math.max(first.sigmaM() * first.sigmaM(), 0.04), 0.0}, {0.0, 0.08 * 0.08}};
        double[][] filteredStates = new double[count][];
        double[][][] filteredCovariances = new double[count][][];
        double[][] predictedStates = new double[count][];
        double[][][] predictedCovariances = new double[count][][];
        double[][][] transitions = new double[count][][];
        String[] sources = new String[count];
        Arrays.fill(sources, "predicted");

        for (int index = 0; index < firstIndex; index++) {
            filteredStates[index] = copy(state);
            filteredCovariances[index] = new double[][]{{Math.max(1.0, covariance[0][0]), 0.0}, {0.0, 0.15 * 0.15}};
            predictedStates[index] = copy(filteredStates[index]);
            predictedCovariances[index] = copy(filteredCovariances[index]);
            transitions[index] = identity();
            sources[index] = "backfilled_before_first_measurement";
        }

        for (int index = firstIndex; index < count; index++) {
            Prediction prediction = index == firstIndex
                    ? new Prediction(copy(state), copy(covariance), identity())
                    : predictState(state, covariance, nodesS[index] - nodesS[index - 1]);
            predictedStates[index] = copy(prediction.state());
            predictedCovariances[index] = copy(prediction.covariance());
            transitions[index] = copy(prediction.transition());
            state = prediction.state();
            covariance = prediction.covariance();
            if (!byNode.get(index).isEmpty()) {
                List<DepthObservation> nodeObservations = new ArrayList<>(byNode.get(index));
                nodeObservations.sort(Comparator.comparingDouble(DepthObservation::captureTimeMs));
                boolean center = false;
                boolean crossing = false;
                for (DepthObservation observation : nodeObservations) {
                    Update update = robustUpdate(state, covariance, observation.calibratedDepthM(), observation.sigmaM());
                    state = update.state();
                    covariance = update.covariance();
                    center |= "center_0".equals(observation.signalContext());
                    crossing |= "crossing_recent".equals(observation.signalContext());
                }
                sources[index] = center ? "measured_center_0" : crossing ? "measured_crossing" : "measured_side";
            }
            filteredStates[index] = copy(state);
            filteredCovariances[index] = copy(covariance);
        }

        double[][] smoothedStates = new double[count][];
        double[][][] smoothedCovariances = new double[count][][];
        for (int index = 0; index < count; index++) {
            smoothedStates[index] = copy(filteredStates[index]);
            smoothedCovariances[index] = copy(filteredCovariances[index]);
        }
        for (int index = count - 2; index >= firstIndex; index--) {
            double[][] smoothingGain = multiply(
                    multiply(filteredCovariances[index], transpose(transitions[index + 1])),
                    inverse(predictedCovariances[index + 1])
            );
            double[] stateDelta = subtract(smoothedStates[index + 1], predictedStates[index + 1]);
            smoothedStates[index] = add(filteredStates[index], multiply(smoothingGain, stateDelta));
            double[][] covarianceDelta = subtract(smoothedCovariances[index + 1], predictedCovariances[index + 1]);
            smoothedCovariances[index] = add(
                    filteredCovariances[index],
                    multiply(multiply(smoothingGain, covarianceDelta), transpose(smoothingGain))
            );
            smoothedStates[index][0] = clamp(smoothedStates[index][0], DEPTH_MIN_M, DEPTH_MAX_M);
        }
        double[] finalDepth = new double[count];
        double[] finalSigma = new double[count];
        for (int index = 0; index < count; index++) {
            finalDepth[index] = smoothedStates[index][0];
            finalSigma[index] = Math.sqrt(Math.max(smoothedCovariances[index][0][0], 0.0));
        }
        return new FilterResult(finalDepth, finalSigma, measurementCount, sources);
    }

    private static Prediction predictState(double[] state, double[][] covariance, double distance) {
        double[][] transition = {{1.0, distance}, {0.0, 1.0}};
        double d2 = distance * distance;
        double[][] processNoise = {
                {SLOPE_RANDOM_WALK_Q * distance * d2 / 3.0, SLOPE_RANDOM_WALK_Q * d2 / 2.0},
                {SLOPE_RANDOM_WALK_Q * d2 / 2.0, SLOPE_RANDOM_WALK_Q * Math.max(distance, 1e-9)}
        };
        return new Prediction(
                multiply(transition, state),
                add(multiply(multiply(transition, covariance), transpose(transition)), processNoise),
                transition
        );
    }

    private static Update robustUpdate(double[] state, double[][] covariance, double value, double sigma) {
        double variance = sigma * sigma;
        double innovation = value - state[0];
        double innovationVariance = covariance[0][0] + variance;
        double normalized = Math.abs(innovation) / Math.max(Math.sqrt(innovationVariance), 1e-9);
        if (normalized > INNOVATION_GATE_SIGMA) {
            double ratio = normalized / INNOVATION_GATE_SIGMA;
            variance *= Math.min(MAX_R_INFLATION, ratio * ratio);
            innovationVariance = covariance[0][0] + variance;
        }
        double[] gain = {covariance[0][0] / innovationVariance, covariance[1][0] / innovationVariance};
        double[] updatedState = {state[0] + gain[0] * innovation, state[1] + gain[1] * innovation};
        double[][] left = {{1.0 - gain[0], 0.0}, {-gain[1], 1.0}};
        double[][] updatedCovariance = add(
                multiply(multiply(left, covariance), transpose(left)),
                new double[][]{
                        {gain[0] * variance * gain[0], gain[0] * variance * gain[1]},
                        {gain[1] * variance * gain[0], gain[1] * variance * gain[1]}
                }
        );
        updatedState[0] = clamp(updatedState[0], DEPTH_MIN_M, DEPTH_MAX_M);
        return new Update(updatedState, updatedCovariance);
    }

    private static String confidenceLabel(double sigma, String source) {
        if ("measured_center_0".equals(source) && sigma <= 0.35) return "high";
        if (sigma <= 0.35) return "medium_high";
        if (sigma <= 0.60) return "medium";
        return "low";
    }

    private static double distance(Point2 left, Point2 right) {
        return Math.hypot(left.x() - right.x(), left.y() - right.y());
    }

    private static double clamp(double value, double minimum, double maximum) {
        return Math.max(minimum, Math.min(maximum, value));
    }

    private static double[] copy(double[] value) {
        return Arrays.copyOf(value, value.length);
    }

    private static double[][] copy(double[][] value) {
        return new double[][]{copy(value[0]), copy(value[1])};
    }

    private static double[][] identity() {
        return new double[][]{{1.0, 0.0}, {0.0, 1.0}};
    }

    private static double[] add(double[] left, double[] right) {
        return new double[]{left[0] + right[0], left[1] + right[1]};
    }

    private static double[] subtract(double[] left, double[] right) {
        return new double[]{left[0] - right[0], left[1] - right[1]};
    }

    private static double[][] add(double[][] left, double[][] right) {
        return new double[][]{
                {left[0][0] + right[0][0], left[0][1] + right[0][1]},
                {left[1][0] + right[1][0], left[1][1] + right[1][1]}
        };
    }

    private static double[][] subtract(double[][] left, double[][] right) {
        return new double[][]{
                {left[0][0] - right[0][0], left[0][1] - right[0][1]},
                {left[1][0] - right[1][0], left[1][1] - right[1][1]}
        };
    }

    private static double[] multiply(double[][] matrix, double[] vector) {
        return new double[]{
                matrix[0][0] * vector[0] + matrix[0][1] * vector[1],
                matrix[1][0] * vector[0] + matrix[1][1] * vector[1]
        };
    }

    private static double[][] multiply(double[][] left, double[][] right) {
        return new double[][]{
                {
                        left[0][0] * right[0][0] + left[0][1] * right[1][0],
                        left[0][0] * right[0][1] + left[0][1] * right[1][1]
                },
                {
                        left[1][0] * right[0][0] + left[1][1] * right[1][0],
                        left[1][0] * right[0][1] + left[1][1] * right[1][1]
                }
        };
    }

    private static double[][] transpose(double[][] matrix) {
        return new double[][]{{matrix[0][0], matrix[1][0]}, {matrix[0][1], matrix[1][1]}};
    }

    private static double[][] inverse(double[][] matrix) {
        double determinant = matrix[0][0] * matrix[1][1] - matrix[0][1] * matrix[1][0];
        if (Math.abs(determinant) < 1e-15) {
            // Covariances are positive definite in normal operation; a small diagonal
            // regularization keeps a degenerate input deterministic without a library dependency.
            matrix = copy(matrix);
            matrix[0][0] += 1e-12;
            matrix[1][1] += 1e-12;
            determinant = matrix[0][0] * matrix[1][1] - matrix[0][1] * matrix[1][0];
        }
        return new double[][]{
                {matrix[1][1] / determinant, -matrix[0][1] / determinant},
                {-matrix[1][0] / determinant, matrix[0][0] / determinant}
        };
    }

    interface TimedValue {
        double timestampMs();
    }

    record Point2(double x, double y) {}

    private record TimedPose(double timestampMs, double x, double y) implements TimedValue {}

    private record TimedSignal(double timestampMs, int signal) implements TimedValue {}

    private record TimedPoint(double timestampMs, double x, double y) implements TimedValue {}

    private record DepthEvent(
            double receiptTimeMs,
            double rawDepthCm,
            Double captureTimeMs,
            Double backendReceiveTimeMs
    ) {}

    record DepthObservation(
            String source,
            double receiptTimeMs,
            double captureTimeMs,
            double rawDepthCm,
            double calibratedDepthM,
            double robotX,
            double robotY,
            double projectedX,
            double projectedY,
            double sM,
            double lateralDistanceM,
            Integer signal,
            String signalContext,
            double sigmaM,
            String observationChannel,
            boolean acceptedPhysicalRange,
            boolean deepZone
    ) {
        Map<String, Object> toMap() {
            Map<String, Object> value = new LinkedHashMap<>();
            value.put("source", source);
            value.put("receipt_time_ms", receiptTimeMs);
            value.put("capture_time_ms", captureTimeMs);
            value.put("raw_depth_cm", rawDepthCm);
            value.put("calibrated_depth_m", calibratedDepthM);
            value.put("robot_x", robotX);
            value.put("robot_y", robotY);
            value.put("projected_x", projectedX);
            value.put("projected_y", projectedY);
            value.put("s_m", sM);
            value.put("lateral_distance_m", lateralDistanceM);
            value.put("signal", signal);
            value.put("signal_context", signalContext);
            value.put("sigma_m", sigmaM);
            value.put("observation_channel", observationChannel);
            value.put("accepted_physical_range", acceptedPhysicalRange);
            value.put("deep_zone", deepZone);
            return value;
        }
    }

    private record SignalContext(String name, Integer signal, double sigmaM) {}

    private record Calibration(double depthM, double sigmaM, String channel) {}

    private record Projection(double sM, double lateralM, Point2 point) {}

    private record ResampledPath(List<Point2> points, double[] sValues) {}

    private record FilterResult(double[] finalDepth, double[] finalSigma, int[] measurementCount, String[] sourceCode) {}

    private record Prediction(double[] state, double[][] covariance, double[][] transition) {}

    private record Update(double[] state, double[][] covariance) {}
}
