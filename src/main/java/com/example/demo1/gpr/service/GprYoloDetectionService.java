package com.example.demo1.gpr.service;

import java.io.IOException;
import java.io.InputStream;
import java.nio.FloatBuffer;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Comparator;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import org.springframework.beans.factory.DisposableBean;
import org.springframework.beans.factory.annotation.Value;
import org.springframework.core.io.Resource;
import org.springframework.core.io.ResourceLoader;
import org.springframework.stereotype.Service;

import com.example.demo1.gpr.model.GprDetection;

import ai.onnxruntime.OnnxTensor;
import ai.onnxruntime.OrtEnvironment;
import ai.onnxruntime.OrtException;
import ai.onnxruntime.OrtSession;

/**
 * 使用训练好的 YOLO11 ONNX 模型识别偏移前 GPR 双曲线。
 *
 * <p>模型输入与训练 PNG 保持一致：rawData -> 绝对值 95.5% 分位数截断
 * -> 线性灰度 -> 居中 letterbox 到 640x640。模型输出在后端完成置信度过滤、
 * NMS 和原始矩阵坐标反算，前端只负责绘制。</p>
 */
@Service
public class GprYoloDetectionService implements DisposableBean {

    private static final Logger LOGGER = LoggerFactory.getLogger(GprYoloDetectionService.class);
    private static final float LETTERBOX_GRAY = 114.0f / 255.0f;
    private static final String[] CLASS_NAMES = {"cable"};

    private final OrtEnvironment environment = OrtEnvironment.getEnvironment();
    private final Resource modelResource;
    private final boolean enabled;
    private final int inputSize;
    private final double confidenceThreshold;
    private final double iouThreshold;
    private final int maxDetections;
    private final double grayscalePercentile;

    private volatile OrtSession session;
    private volatile String inputName;
    private volatile boolean initializationAttempted;

    public GprYoloDetectionService(
            ResourceLoader resourceLoader,
            @Value("${gpr.yolo.model:classpath:/models/gpr-hyperbola-yolo11n.onnx}") String modelLocation,
            @Value("${gpr.yolo.enabled:true}") boolean enabled,
            @Value("${gpr.yolo.input-size:640}") int inputSize,
            @Value("${gpr.yolo.confidence-threshold:0.50}") double confidenceThreshold,
            @Value("${gpr.yolo.iou-threshold:0.45}") double iouThreshold,
            @Value("${gpr.yolo.max-detections:50}") int maxDetections,
            @Value("${gpr.yolo.grayscale-percentile:0.955}") double grayscalePercentile
    ) {
        this.modelResource = resourceLoader.getResource(modelLocation);
        this.enabled = enabled;
        this.inputSize = Math.max(32, inputSize);
        this.confidenceThreshold = clamp(confidenceThreshold, 0.0, 1.0);
        this.iouThreshold = clamp(iouThreshold, 0.0, 1.0);
        this.maxDetections = Math.max(1, maxDetections);
        this.grayscalePercentile = clamp(grayscalePercentile, 0.5, 0.999);
    }

    /**
     * 识别一个完整 B-Scan。模型不可用时返回空列表，不影响原有偏移处理。
     */
    public List<GprDetection> detect(double[][] rawData) {
        if (!enabled || !isValidMatrix(rawData)) {
            return List.of();
        }

        try {
            OrtSession activeSession = ensureSession();
            PreprocessedInput input = preprocess(rawData);
            long[] shape = {1, 3, inputSize, inputSize};

            try (OnnxTensor tensor = OnnxTensor.createTensor(
                    environment,
                    FloatBuffer.wrap(input.tensorData),
                    shape
            )) {
                Map<String, OnnxTensor> inputs = new HashMap<>();
                inputs.put(inputName, tensor);
                try (OrtSession.Result result = activeSession.run(inputs)) {
                    if (result.size() == 0) {
                        return List.of();
                    }
                    List<GprDetection> detections = decodeOutput(result.get(0).getValue(), input);
                    try {
                        GprApexLocator.locateAll(rawData, detections);
                    } catch (RuntimeException apexError) {
                        LOGGER.warn("GPR 顶点定位失败，保留 YOLO 检测框: {}", apexError.getMessage());
                    }
                    return detections;
                }
            }
        } catch (Exception ex) {
            LOGGER.warn("GPR YOLO 识别失败，本段将不显示自动识别框: {}", ex.getMessage());
            return List.of();
        }
    }

    private synchronized OrtSession ensureSession() throws IOException, OrtException {
        if (session != null) {
            return session;
        }
        if (initializationAttempted) {
            throw new IllegalStateException("YOLO ONNX 模型初始化此前已失败");
        }
        initializationAttempted = true;

        if (!modelResource.exists()) {
            throw new IOException("找不到 YOLO 模型: " + modelResource.getDescription());
        }

        byte[] modelBytes;
        try (InputStream inputStream = modelResource.getInputStream()) {
            modelBytes = inputStream.readAllBytes();
        }

        try (OrtSession.SessionOptions options = new OrtSession.SessionOptions()) {
            options.setOptimizationLevel(OrtSession.SessionOptions.OptLevel.ALL_OPT);
            session = environment.createSession(modelBytes, options);
        }
        inputName = session.getInputNames().iterator().next();
        LOGGER.info(
                "GPR YOLO 模型已加载: {}, input={}, outputs={}",
                modelResource.getDescription(),
                inputName,
                session.getOutputNames()
        );
        return session;
    }

    private PreprocessedInput preprocess(double[][] matrix) {
        int sourceHeight = matrix.length;
        int sourceWidth = matrix[0].length;
        double absLimit = computeRobustAbsLimit(matrix);

        float[][] grayscale = new float[sourceHeight][sourceWidth];
        for (int y = 0; y < sourceHeight; y++) {
            for (int x = 0; x < sourceWidth; x++) {
                double normalized = (matrix[y][x] + absLimit) / (2.0 * absLimit);
                int gray = (int) Math.round(clamp(normalized, 0.0, 1.0) * 255.0);
                grayscale[y][x] = gray / 255.0f;
            }
        }

        double scale = Math.min(
                (double) inputSize / sourceWidth,
                (double) inputSize / sourceHeight
        );
        int resizedWidth = Math.max(1, (int) Math.round(sourceWidth * scale));
        int resizedHeight = Math.max(1, (int) Math.round(sourceHeight * scale));
        double padX = (inputSize - resizedWidth) / 2.0;
        double padY = (inputSize - resizedHeight) / 2.0;
        int left = (int) Math.round(padX - 0.1);
        int top = (int) Math.round(padY - 0.1);

        int planeSize = inputSize * inputSize;
        float[] tensorData = new float[3 * planeSize];
        Arrays.fill(tensorData, LETTERBOX_GRAY);

        for (int dy = 0; dy < resizedHeight; dy++) {
            double sourceY = ((dy + 0.5) * sourceHeight / resizedHeight) - 0.5;
            int y0 = clampIndex((int) Math.floor(sourceY), sourceHeight);
            int y1 = clampIndex(y0 + 1, sourceHeight);
            double wy = clamp(sourceY - Math.floor(sourceY), 0.0, 1.0);

            for (int dx = 0; dx < resizedWidth; dx++) {
                double sourceX = ((dx + 0.5) * sourceWidth / resizedWidth) - 0.5;
                int x0 = clampIndex((int) Math.floor(sourceX), sourceWidth);
                int x1 = clampIndex(x0 + 1, sourceWidth);
                double wx = clamp(sourceX - Math.floor(sourceX), 0.0, 1.0);

                double topValue = grayscale[y0][x0] * (1.0 - wx) + grayscale[y0][x1] * wx;
                double bottomValue = grayscale[y1][x0] * (1.0 - wx) + grayscale[y1][x1] * wx;
                float value = (float) (topValue * (1.0 - wy) + bottomValue * wy);

                int targetX = left + dx;
                int targetY = top + dy;
                if (targetX < 0 || targetX >= inputSize || targetY < 0 || targetY >= inputSize) {
                    continue;
                }
                int planeIndex = targetY * inputSize + targetX;
                tensorData[planeIndex] = value;
                tensorData[planeSize + planeIndex] = value;
                tensorData[2 * planeSize + planeIndex] = value;
            }
        }

        return new PreprocessedInput(
                tensorData,
                sourceWidth,
                sourceHeight,
                scale,
                padX,
                padY
        );
    }

    private List<GprDetection> decodeOutput(Object outputValue, PreprocessedInput input) {
        if (!(outputValue instanceof float[][][] output) || output.length == 0 || output[0].length == 0) {
            throw new IllegalArgumentException("不支持的 YOLO 输出类型: " + outputValue.getClass().getName());
        }

        float[][] predictions = output[0];
        boolean channelFirst = predictions.length <= predictions[0].length;
        int featureCount = channelFirst ? predictions.length : predictions[0].length;
        int candidateCount = channelFirst ? predictions[0].length : predictions.length;
        if (featureCount < 5) {
            throw new IllegalArgumentException("YOLO 输出特征数不足: " + featureCount);
        }

        List<GprDetection> candidates = new ArrayList<>();
        for (int candidateIndex = 0; candidateIndex < candidateCount; candidateIndex++) {
            double centerX = outputValue(predictions, channelFirst, 0, candidateIndex);
            double centerY = outputValue(predictions, channelFirst, 1, candidateIndex);
            double width = outputValue(predictions, channelFirst, 2, candidateIndex);
            double height = outputValue(predictions, channelFirst, 3, candidateIndex);

            int bestClassId = 0;
            double bestConfidence = -Double.MAX_VALUE;
            for (int featureIndex = 4; featureIndex < featureCount; featureIndex++) {
                double confidence = outputValue(predictions, channelFirst, featureIndex, candidateIndex);
                if (confidence > bestConfidence) {
                    bestConfidence = confidence;
                    bestClassId = featureIndex - 4;
                }
            }
            if (!Double.isFinite(bestConfidence) || bestConfidence < confidenceThreshold) {
                continue;
            }

            double x1 = (centerX - width / 2.0 - input.padX) / input.scale;
            double y1 = (centerY - height / 2.0 - input.padY) / input.scale;
            double x2 = (centerX + width / 2.0 - input.padX) / input.scale;
            double y2 = (centerY + height / 2.0 - input.padY) / input.scale;

            x1 = clamp(x1, 0.0, input.sourceWidth - 1.0);
            y1 = clamp(y1, 0.0, input.sourceHeight - 1.0);
            x2 = clamp(x2, 0.0, input.sourceWidth - 1.0);
            y2 = clamp(y2, 0.0, input.sourceHeight - 1.0);
            if (x2 <= x1 || y2 <= y1) {
                continue;
            }

            String label = bestClassId >= 0 && bestClassId < CLASS_NAMES.length
                    ? CLASS_NAMES[bestClassId]
                    : "class_" + bestClassId;
            candidates.add(new GprDetection(
                    bestClassId,
                    label,
                    bestConfidence,
                    x1,
                    y1,
                    x2,
                    y2
            ));
        }

        return nonMaximumSuppression(candidates);
    }

    private List<GprDetection> nonMaximumSuppression(List<GprDetection> candidates) {
        candidates.sort(Comparator.comparingDouble(GprDetection::getConfidence).reversed());
        List<GprDetection> selected = new ArrayList<>();
        for (GprDetection candidate : candidates) {
            boolean suppressed = false;
            for (GprDetection existing : selected) {
                if (candidate.getClassId() == existing.getClassId()
                        && intersectionOverUnion(candidate, existing) > iouThreshold) {
                    suppressed = true;
                    break;
                }
            }
            if (!suppressed) {
                selected.add(candidate);
                if (selected.size() >= maxDetections) {
                    break;
                }
            }
        }
        return List.copyOf(selected);
    }

    private double computeRobustAbsLimit(double[][] matrix) {
        int total = matrix.length * matrix[0].length;
        double[] absoluteValues = new double[total];
        int count = 0;
        double maxAbs = 0.0;

        for (double[] row : matrix) {
            for (double value : row) {
                if (!Double.isFinite(value)) {
                    continue;
                }
                double absValue = Math.abs(value);
                absoluteValues[count++] = absValue;
                maxAbs = Math.max(maxAbs, absValue);
            }
        }
        if (count == 0 || maxAbs == 0.0) {
            return 1e-9;
        }

        Arrays.sort(absoluteValues, 0, count);
        int percentileIndex = (int) Math.floor((count - 1) * grayscalePercentile);
        double limit = absoluteValues[percentileIndex];
        return limit > 0.0 ? limit : Math.max(maxAbs * 0.6, 1e-9);
    }

    private double intersectionOverUnion(GprDetection a, GprDetection b) {
        double intersectionWidth = Math.max(0.0, Math.min(a.getX2(), b.getX2()) - Math.max(a.getX1(), b.getX1()));
        double intersectionHeight = Math.max(0.0, Math.min(a.getY2(), b.getY2()) - Math.max(a.getY1(), b.getY1()));
        double intersection = intersectionWidth * intersectionHeight;
        double areaA = Math.max(0.0, a.getX2() - a.getX1()) * Math.max(0.0, a.getY2() - a.getY1());
        double areaB = Math.max(0.0, b.getX2() - b.getX1()) * Math.max(0.0, b.getY2() - b.getY1());
        double union = areaA + areaB - intersection;
        return union > 0.0 ? intersection / union : 0.0;
    }

    private double outputValue(float[][] output, boolean channelFirst, int feature, int candidate) {
        return channelFirst ? output[feature][candidate] : output[candidate][feature];
    }

    private boolean isValidMatrix(double[][] matrix) {
        if (matrix == null || matrix.length == 0 || matrix[0] == null || matrix[0].length == 0) {
            return false;
        }
        int width = matrix[0].length;
        for (double[] row : matrix) {
            if (row == null || row.length != width) {
                return false;
            }
        }
        return true;
    }

    private static int clampIndex(int index, int size) {
        return Math.max(0, Math.min(size - 1, index));
    }

    private static double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }

    @Override
    public synchronized void destroy() throws OrtException {
        if (session != null) {
            session.close();
            session = null;
        }
    }

    private static class PreprocessedInput {
        private final float[] tensorData;
        private final int sourceWidth;
        private final int sourceHeight;
        private final double scale;
        private final double padX;
        private final double padY;

        private PreprocessedInput(
                float[] tensorData,
                int sourceWidth,
                int sourceHeight,
                double scale,
                double padX,
                double padY
        ) {
            this.tensorData = tensorData;
            this.sourceWidth = sourceWidth;
            this.sourceHeight = sourceHeight;
            this.scale = scale;
            this.padX = padX;
            this.padY = padY;
        }
    }
}
