package com.example.demo1.gpr.service;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertNull;

import java.util.List;

import org.junit.jupiter.api.Test;
import org.springframework.core.io.DefaultResourceLoader;

import com.example.demo1.gpr.model.GprDetection;

class GprYoloDetectionServiceTest {

    @Test
    void decodesPoseKeypointBackToSourceCoordinates() throws Exception {
        GprYoloDetectionService service = service(0.25);
        try {
            float[][][] output = emptyChannelFirstOutput(10);
            setCandidate(output, 0, 320, 320, 128, 64, 0.90, 300, 290, 0.80);
            setCandidate(output, 1, 100, 100, 20, 20, 0.10, 100, 100, 0.90);

            List<GprDetection> detections = service.decodeOutput(
                    output,
                    input(320, 256, 2.0, 0.0, 64.0)
            );

            assertEquals(1, detections.size());
            GprDetection detection = detections.get(0);
            assertEquals(128.0, detection.getX1(), 1e-6);
            assertEquals(112.0, detection.getY1(), 1e-6);
            assertEquals(192.0, detection.getX2(), 1e-6);
            assertEquals(144.0, detection.getY2(), 1e-6);
            assertEquals(150.0, detection.getApexCol(), 1e-6);
            assertEquals(113.0, detection.getApexRow(), 1e-6);
            assertEquals(0.80, detection.getApexScore(), 1e-6);
            assertEquals("pose", detection.getApexMethod());
        } finally {
            service.destroy();
        }
    }

    @Test
    void keepsDetectionWhenKeypointConfidenceIsTooLow() throws Exception {
        GprYoloDetectionService service = service(0.50);
        try {
            float[][][] output = emptyChannelFirstOutput(10);
            setCandidate(output, 0, 320, 320, 128, 64, 0.90, 300, 290, 0.20);

            List<GprDetection> detections = service.decodeOutput(
                    output,
                    input(320, 256, 2.0, 0.0, 64.0)
            );

            assertEquals(1, detections.size());
            assertNull(detections.get(0).getApexCol());
            assertNull(detections.get(0).getApexRow());
            assertNull(detections.get(0).getApexScore());
            assertNull(detections.get(0).getApexMethod());
        } finally {
            service.destroy();
        }
    }

    @Test
    void loadsPackagedPoseOnnxModel() throws Exception {
        GprYoloDetectionService service = service(
                0.25,
                "classpath:/models/gpr-hyperbola-apex-yolo11n-pose.onnx"
        );
        try {
            assertFalse(service.ensureSession().getInputNames().isEmpty());
            assertFalse(service.ensureSession().getOutputNames().isEmpty());
        } finally {
            service.destroy();
        }
    }

    private static GprYoloDetectionService service(double keypointThreshold) {
        return service(keypointThreshold, "classpath:/unused.onnx");
    }

    private static GprYoloDetectionService service(
            double keypointThreshold,
            String modelLocation
    ) {
        return new GprYoloDetectionService(
                new DefaultResourceLoader(),
                modelLocation,
                true,
                640,
                0.50,
                keypointThreshold,
                0.45,
                50,
                0.955
        );
    }

    private static GprYoloDetectionService.PreprocessedInput input(
            int width,
            int height,
            double scale,
            double padX,
            double padY
    ) {
        return new GprYoloDetectionService.PreprocessedInput(
                new float[0], width, height, scale, padX, padY
        );
    }

    private static float[][][] emptyChannelFirstOutput(int candidates) {
        return new float[1][8][candidates];
    }

    private static void setCandidate(
            float[][][] output,
            int candidate,
            float centerX,
            float centerY,
            float width,
            float height,
            double confidence,
            float apexX,
            float apexY,
            double apexConfidence
    ) {
        output[0][0][candidate] = centerX;
        output[0][1][candidate] = centerY;
        output[0][2][candidate] = width;
        output[0][3][candidate] = height;
        output[0][4][candidate] = (float) confidence;
        output[0][5][candidate] = apexX;
        output[0][6][candidate] = apexY;
        output[0][7][candidate] = (float) apexConfidence;
    }
}
