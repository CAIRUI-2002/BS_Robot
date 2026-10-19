package com.example.demo1.service;

import org.junit.jupiter.api.Test;

import java.util.List;
import java.util.Map;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotNull;

class PipelineDepthEstimatorTest {

    @Test
    void javaEstimatorMatchesValidatedReferenceSequence() {
        PipelineDepthEstimator estimator = new PipelineDepthEstimator();
        estimator.addCablePoint(900.0, 0.0, 0.0);
        estimator.addCablePoint(901.0, 2.0, 0.0);
        estimator.addPose(1000.0, 0.0, 0.2);
        estimator.addPose(1100.0, 0.5, 0.2);
        estimator.addPose(1200.0, 1.0, 0.2);
        estimator.addPose(1300.0, 1.5, 0.2);
        estimator.addPose(1400.0, 2.0, 0.2);
        estimator.addSignal(1000.0, 1);
        estimator.addSignal(1150.0, 2);

        assertObservation(estimator.addDepth(1000.0, 100.0, null, null), 0.0, 0.9190287548221776, "side", "single_fast");
        assertObservation(estimator.addDepth(1100.0, 120.0, null, null), 0.5, 0.9736907596564581, "side", "single_fast");
        assertObservation(estimator.addDepth(1200.0, 140.0, null, null), 1.0, 1.0199068262264022, "crossing_recent", "single_fast");
        assertObservation(estimator.addDepth(1300.0, 160.0, null, null), 1.5, 0.7911516879822975, "crossing_recent", "rolling_q25_4");
        estimator.addSignal(1400.0, 0);
        assertObservation(estimator.addDepth(1400.0, 180.0, null, null), 2.0, 1.0952536515562248, "center_0", "single_center_0");

        List<Map<String, Object>> rows = estimator.finalizeMap();
        assertEquals(21, rows.size());
        assertNode(rows.get(0), 0.9842306703593449, 0.20204172546073382, "measured_side", 1);
        assertNode(rows.get(10), 0.9928705919516835, 0.17173506041910533, "measured_crossing", 1);
        assertNode(rows.get(20), 1.0035632337127145, 0.17647798173238108, "measured_center_0", 1);
        assertEquals(true, rows.get(20).get("finalized"));
    }

    @Test
    void depthWaitsUntilPathAndPoseAreAvailable() {
        PipelineDepthEstimator estimator = new PipelineDepthEstimator();
        estimator.addDepth(1000.0, 100.0, null, null);
        assertEquals(1, estimator.pendingCount());
        estimator.addCablePoint(900.0, 0.0, 0.0);
        estimator.addCablePoint(901.0, 1.0, 0.0);
        assertEquals(1, estimator.pendingCount());
        estimator.addPose(1000.0, 0.5, 0.0);
        estimator.replaceCablePath(1100.0, List.of(
                new PipelineDepthEstimator.Point2(0.0, 0.0),
                new PipelineDepthEstimator.Point2(1.0, 0.0)
        ));
        assertEquals(0, estimator.pendingCount());
        assertEquals(11, estimator.getMap().size());
    }

    private static void assertObservation(
            PipelineDepthEstimator.DepthObservation observation,
            double expectedS,
            double expectedDepth,
            String expectedContext,
            String expectedChannel
    ) {
        assertNotNull(observation);
        assertEquals(expectedS, observation.sM(), 1e-12);
        assertEquals(expectedDepth, observation.calibratedDepthM(), 1e-12);
        assertEquals(expectedContext, observation.signalContext());
        assertEquals(expectedChannel, observation.observationChannel());
    }

    private static void assertNode(
            Map<String, Object> node,
            double expectedDepth,
            double expectedSigma,
            String expectedSource,
            int expectedCount
    ) {
        assertEquals(expectedDepth, ((Number) node.get("depth_m")).doubleValue(), 1e-12);
        assertEquals(expectedSigma, ((Number) node.get("uncertainty_1sigma_m")).doubleValue(), 1e-12);
        assertEquals(expectedSource, node.get("source"));
        assertEquals(expectedCount, ((Number) node.get("measurement_count_at_node")).intValue());
    }
}
