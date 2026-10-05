package com.example.demo1.gpr.service;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

class GprRtkProfilePoseTest {

    @Test
    void keepsDztTraceSpacingAndRejectsRtkLateralOutliers() {
        int sampleCount = 101;
        double[] x = new double[sampleCount];
        double[] y = new double[sampleCount];
        double angle = Math.toRadians(28.0);
        double directionX = Math.cos(angle);
        double directionY = Math.sin(angle);
        double normalX = -directionY;
        double normalY = directionX;

        for (int i = 0; i < sampleCount; i++) {
            double along = (i - 50) * 0.02;
            double noise = Math.sin(i * 0.37) * 0.012;
            x[i] = 10.0 + along * directionX + noise * normalX;
            y[i] = 20.0 + along * directionY + noise * normalY;
        }
        x[12] += normalX * 1.2;
        y[12] += normalY * 1.2;
        x[88] -= normalX * 0.9;
        y[88] -= normalY * 0.9;

        GprRtkProfilePose.RigidPath pose = GprRtkProfilePose.fit(x, y, 81, 0.01);

        assertNotNull(pose);
        assertEquals(0.8, pose.length(), 1e-12);
        assertTrue(pose.inlierCount() < pose.sampleCount());
        assertTrue(pose.rmseM() < 0.02);
        for (int i = 1; i < pose.traceX().length; i++) {
            assertEquals(0.01, Math.hypot(
                    pose.traceX()[i] - pose.traceX()[i - 1],
                    pose.traceY()[i] - pose.traceY()[i - 1]
            ), 1e-10);
        }
        double fittedDirectionX = pose.traceX()[80] - pose.traceX()[0];
        double fittedDirectionY = pose.traceY()[80] - pose.traceY()[0];
        assertTrue(fittedDirectionX * directionX + fittedDirectionY * directionY > 0);
    }

    @Test
    void preservesTheAcquisitionDirectionProvidedByRtk() {
        double[] x = {4, 3, 2, 1, 0};
        double[] y = {1, 1, 1, 1, 1};

        GprRtkProfilePose.RigidPath pose = GprRtkProfilePose.fit(x, y, 5, 0.25);

        assertNotNull(pose);
        assertTrue(pose.traceX()[0] > pose.traceX()[4]);
        assertEquals(1.0, pose.traceY()[0], 1e-12);
        assertEquals(1.0, pose.traceY()[4], 1e-12);
        assertEquals(1.0, pose.length(), 1e-12);
    }
}
