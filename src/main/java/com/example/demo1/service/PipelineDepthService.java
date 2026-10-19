package com.example.demo1.service;

import com.example.demo1.model.CablePoint;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import org.springframework.messaging.simp.SimpMessagingTemplate;
import org.springframework.stereotype.Service;

import java.util.ArrayList;
import java.util.HashSet;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.Set;
import java.util.UUID;

/**
 * Owns the Java cable-depth estimator for the current inspection session.
 * All methods that touch estimator state are synchronized so REST requests and
 * incoming pipeline measurements cannot mutate the filter concurrently.
 */
@Service
public class PipelineDepthService {

    private static final Logger log = LoggerFactory.getLogger(PipelineDepthService.class);

    private final SimpMessagingTemplate messagingTemplate;
    private final Set<String> seenPointIds = new HashSet<>();
    private final Set<String> seenDepthIds = new HashSet<>();

    private PipelineDepthEstimator estimator;
    private boolean active;
    private String sessionId;
    private List<Map<String, Object>> lastNodes = List.of();

    public PipelineDepthService(SimpMessagingTemplate messagingTemplate) {
        this.messagingTemplate = messagingTemplate;
    }

    public synchronized Map<String, Object> startSession(String requestedSessionId, String reason) {
        sessionId = requestedSessionId == null || requestedSessionId.isBlank()
                ? UUID.randomUUID().toString()
                : requestedSessionId.trim();
        estimator = new PipelineDepthEstimator();
        seenPointIds.clear();
        seenDepthIds.clear();
        lastNodes = List.of();
        active = true;

        Map<String, Object> response = successWithNodes(lastNodes);
        response.put("reason", reason == null ? "inspection" : reason);
        publishDepthMap(response, "start");
        return enrich(response);
    }

    public synchronized Map<String, Object> addPose(double xM, double yM) {
        if (!active || estimator == null) return ignored("no active depth session");
        if (!Double.isFinite(xM) || !Double.isFinite(yM)) return failure("invalid pose coordinates");
        try {
            estimator.addPose(System.currentTimeMillis(), xM, yM);
            return enrich(success());
        } catch (RuntimeException error) {
            return estimatorFailure("pose", error);
        }
    }

    public synchronized Map<String, Object> addSignal(int signal, long receiptTimeMs) {
        if (!active || estimator == null) return ignored("no active depth session");
        if (signal < 0 || signal > 2) return ignored("signal outside 0/1/2");
        try {
            estimator.addSignal(receiptTimeMs, signal);
            return enrich(success());
        } catch (RuntimeException error) {
            return estimatorFailure("signal", error);
        }
    }

    public synchronized Map<String, Object> addCablePoint(String pointId, double xM, double yM) {
        if (!active || estimator == null) return ignored("no active depth session");
        if (!Double.isFinite(xM) || !Double.isFinite(yM)) return failure("invalid cable-point coordinates");
        if (pointId != null && !pointId.isBlank() && seenPointIds.contains(pointId)) {
            Map<String, Object> duplicate = successWithNodes(estimator.getMap());
            duplicate.put("duplicate", true);
            publishDepthMap(duplicate, "cable_point");
            return enrich(duplicate);
        }
        try {
            estimator.addCablePoint(System.currentTimeMillis(), xM, yM);
            if (pointId != null && !pointId.isBlank()) seenPointIds.add(pointId);
            Map<String, Object> response = successWithNodes(estimator.getMap());
            publishDepthMap(response, "cable_point");
            return enrich(response);
        } catch (RuntimeException error) {
            return estimatorFailure("cable_point", error);
        }
    }

    public synchronized Map<String, Object> replaceCablePath(List<CablePoint> points) {
        if (!active || estimator == null) return ignored("no active depth session");
        List<PipelineDepthEstimator.Point2> validPoints = new ArrayList<>();
        if (points != null) {
            for (CablePoint point : points) {
                if (point == null || point.getX() == null || point.getY() == null) continue;
                if (!Double.isFinite(point.getX()) || !Double.isFinite(point.getY())) continue;
                validPoints.add(new PipelineDepthEstimator.Point2(point.getX(), point.getY()));
            }
        }
        try {
            estimator.replaceCablePath(System.currentTimeMillis(), validPoints);
            Map<String, Object> response = successWithNodes(estimator.getMap());
            publishDepthMap(response, "replace_path");
            return enrich(response);
        } catch (RuntimeException error) {
            return estimatorFailure("replace_path", error);
        }
    }

    public synchronized Map<String, Object> addDepth(
            String sampleId,
            double rawDepthCm,
            Double captureTimeMs,
            Double backendReceiveTimeMs,
            long receiptTimeMs
    ) {
        if (!active || estimator == null) return ignored("no active depth session");
        if (!Double.isFinite(rawDepthCm) || rawDepthCm <= 0.0) return ignored("invalid raw depth");
        if (sampleId != null && !sampleId.isBlank() && seenDepthIds.contains(sampleId)) {
            Map<String, Object> duplicate = successWithNodes(estimator.getMap());
            duplicate.put("duplicate", true);
            publishDepthMap(duplicate, "depth");
            return enrich(duplicate);
        }
        try {
            PipelineDepthEstimator.DepthObservation observation = estimator.addDepth(
                    receiptTimeMs,
                    rawDepthCm,
                    captureTimeMs,
                    backendReceiveTimeMs
            );
            if (sampleId != null && !sampleId.isBlank()) seenDepthIds.add(sampleId);
            Map<String, Object> response = successWithNodes(estimator.getMap());
            response.put("pending", observation == null);
            response.put("observation", observation == null ? null : observation.toMap());
            publishDepthMap(response, "depth");
            return enrich(response);
        } catch (RuntimeException error) {
            return estimatorFailure("depth", error);
        }
    }

    public synchronized Map<String, Object> finalizeSession() {
        if (!active || estimator == null) {
            Map<String, Object> response = successWithNodes(lastNodes);
            response.put("active", false);
            response.put("sessionId", sessionId);
            return response;
        }
        try {
            Map<String, Object> response = successWithNodes(estimator.finalizeMap());
            response.put("pending", estimator.pendingCount() > 0);
            active = false;
            publishDepthMap(response, "finalize");
            return enrich(response);
        } catch (RuntimeException error) {
            return estimatorFailure("finalize", error);
        }
    }

    public synchronized Map<String, Object> currentMap() {
        if (!active || estimator == null) {
            Map<String, Object> response = successWithNodes(lastNodes);
            response.put("active", false);
            response.put("sessionId", sessionId == null ? "" : sessionId);
            return response;
        }
        try {
            Map<String, Object> response = successWithNodes(estimator.getMap());
            updateLastNodes(response);
            return enrich(response);
        } catch (RuntimeException error) {
            return estimatorFailure("map", error);
        }
    }

    public synchronized Map<String, Object> health() {
        Map<String, Object> response = success();
        response.put("engine", "java");
        response.put("active", active);
        response.put("sessionId", sessionId);
        return response;
    }

    public synchronized boolean isActive() {
        return active;
    }

    private Map<String, Object> estimatorFailure(String operation, RuntimeException error) {
        log.error("Pipeline depth estimator failed for {}", operation, error);
        active = false;
        return enrich(failure("depth estimator unavailable: " + rootMessage(error)));
    }

    private void updateLastNodes(Map<String, Object> response) {
        Object nodes = response.get("nodes");
        if (!(nodes instanceof List<?> values)) return;
        List<Map<String, Object>> copy = new ArrayList<>();
        for (Object value : values) {
            if (value instanceof Map<?, ?> map) {
                Map<String, Object> row = new LinkedHashMap<>();
                for (Map.Entry<?, ?> entry : map.entrySet()) row.put(String.valueOf(entry.getKey()), entry.getValue());
                copy.add(row);
            }
        }
        lastNodes = List.copyOf(copy);
    }

    private void publishDepthMap(Map<String, Object> response, String updateType) {
        if (!Boolean.TRUE.equals(response.get("ok")) || !(response.get("nodes") instanceof List<?>)) return;
        updateLastNodes(response);
        Map<String, Object> payload = new LinkedHashMap<>();
        payload.put("sessionId", sessionId);
        payload.put("updateType", updateType);
        payload.put("active", active);
        payload.put("pending", response.getOrDefault("pending", false));
        payload.put("observation", response.get("observation"));
        payload.put("nodes", lastNodes);
        messagingTemplate.convertAndSend("/topic/pipeline-depth", (Object) payload);
    }

    private Map<String, Object> enrich(Map<String, Object> response) {
        Map<String, Object> enriched = new LinkedHashMap<>(response);
        enriched.put("active", active);
        enriched.put("sessionId", sessionId);
        return enriched;
    }

    private static Map<String, Object> success() {
        Map<String, Object> response = new LinkedHashMap<>();
        response.put("ok", true);
        return response;
    }

    private static Map<String, Object> successWithNodes(List<Map<String, Object>> nodes) {
        Map<String, Object> response = success();
        response.put("nodes", nodes);
        return response;
    }

    private static Map<String, Object> ignored(String reason) {
        Map<String, Object> result = success();
        result.put("ignored", true);
        result.put("reason", reason);
        return result;
    }

    private static Map<String, Object> failure(String message) {
        Map<String, Object> result = new LinkedHashMap<>();
        result.put("ok", false);
        result.put("error", message);
        return result;
    }

    private static String rootMessage(Throwable error) {
        Throwable current = error;
        while (current.getCause() != null) current = current.getCause();
        return current.getMessage() == null ? current.getClass().getSimpleName() : current.getMessage();
    }
}
