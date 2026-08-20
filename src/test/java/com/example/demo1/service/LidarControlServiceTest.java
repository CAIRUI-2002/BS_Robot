package com.example.demo1.service;

import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;
import com.sun.net.httpserver.HttpExchange;
import com.sun.net.httpserver.HttpServer;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.springframework.test.util.ReflectionTestUtils;

import java.io.IOException;
import java.net.InetSocketAddress;
import java.nio.charset.StandardCharsets;
import java.util.Map;
import java.util.concurrent.atomic.AtomicReference;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

class LidarControlServiceTest {

    private final ObjectMapper objectMapper = new ObjectMapper();
    private final AtomicReference<CapturedRequest> capturedRequest = new AtomicReference<>();

    private HttpServer server;
    private LidarControlService service;

    @BeforeEach
    void setUp() throws IOException {
        server = HttpServer.create(new InetSocketAddress("127.0.0.1", 0), 0);
        server.createContext("/", this::handleRequest);
        server.start();

        String baseUrl = "http://127.0.0.1:" + server.getAddress().getPort();
        service = new LidarControlService();
        ReflectionTestUtils.setField(service, "startUrl", baseUrl + "/api/v2/record/start");
        ReflectionTestUtils.setField(service, "stopUrl", baseUrl + "/api/v2/record/stop");
        ReflectionTestUtils.setField(service, "timeoutMs", 1000);
    }

    @AfterEach
    void tearDown() {
        server.stop(0);
    }

    @Test
    void startRecordingSendsStartPyCompatibleJsonPost() throws Exception {
        Map<String, Object> result = service.startRecording();

        CapturedRequest request = capturedRequest.get();
        assertEquals("POST", request.method());
        assertEquals("/api/v2/record/start", request.path());
        assertEquals("application/json", request.accept());
        assertTrue(request.contentType().startsWith("application/json"));

        JsonNode payload = objectMapper.readTree(request.body());
        assertTrue(payload.hasNonNull("project_name"));
        assertFalse(payload.path("scan_resumer").path("enable").asBoolean());
        assertEquals("the project to be resumed",
                payload.path("scan_resumer").path("based_project_name").asText());
        assertEquals(true, result.get("success"));
        assertEquals(200, result.get("status"));
    }

    @Test
    void stopRecordingSendsStopPyCompatibleEmptyPost() throws Exception {
        Map<String, Object> result = service.stopRecording();

        CapturedRequest request = capturedRequest.get();
        assertEquals("POST", request.method());
        assertEquals("/api/v2/record/stop", request.path());
        assertEquals("application/json", request.accept());
        assertEquals(0, request.body().length);
        assertEquals(true, result.get("success"));
        assertEquals(200, result.get("status"));
    }

    private void handleRequest(HttpExchange exchange) throws IOException {
        byte[] requestBody = exchange.getRequestBody().readAllBytes();
        capturedRequest.set(new CapturedRequest(
                exchange.getRequestMethod(),
                exchange.getRequestURI().getPath(),
                exchange.getRequestHeaders().getFirst("Accept"),
                exchange.getRequestHeaders().getFirst("Content-Type"),
                requestBody
        ));

        byte[] responseBody = "{\"ok\":true}".getBytes(StandardCharsets.UTF_8);
        exchange.getResponseHeaders().set("Content-Type", "application/json");
        exchange.sendResponseHeaders(200, responseBody.length);
        try (var response = exchange.getResponseBody()) {
            response.write(responseBody);
        }
    }

    private record CapturedRequest(String method, String path, String accept, String contentType, byte[] body) {
    }
}
