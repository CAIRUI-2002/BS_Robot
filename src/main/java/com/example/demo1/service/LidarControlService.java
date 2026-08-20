package com.example.demo1.service;

import com.fasterxml.jackson.databind.ObjectMapper;
import org.springframework.beans.factory.annotation.Value;
import org.springframework.stereotype.Service;

import java.io.IOException;
import java.io.OutputStream;
import java.net.HttpURLConnection;
import java.net.Proxy;
import java.net.URL;
import java.nio.charset.StandardCharsets;
import java.time.LocalDateTime;
import java.time.format.DateTimeFormatter;
import java.util.LinkedHashMap;
import java.util.Map;

@Service
public class LidarControlService {

    private final ObjectMapper objectMapper = new ObjectMapper();

    @Value("${lidar.record.start-url:http://192.168.19.97:8000/api/v2/record/start}")
    private String startUrl;

    @Value("${lidar.record.stop-url:http://192.168.19.97:8000/api/v2/record/stop}")
    private String stopUrl;

    @Value("${lidar.record.timeout-ms:10000}")
    private int timeoutMs;

    public Map<String, Object> startRecording() throws IOException {
        String projectName = LocalDateTime.now().format(DateTimeFormatter.ofPattern("yyyy-MM-dd_HH-mm-ss"));

        Map<String, Object> payload = new LinkedHashMap<>();
        payload.put("project_name", projectName);

        Map<String, Object> scanResumer = new LinkedHashMap<>();
        scanResumer.put("enable", false);
        scanResumer.put("based_project_name", "the project to be resumed");
        payload.put("scan_resumer", scanResumer);

        byte[] body = objectMapper.writeValueAsBytes(payload);

        DeviceResponse response = executePost(startUrl, body, true, "start");

        Map<String, Object> result = new LinkedHashMap<>();
        result.put("success", true);
        result.put("projectName", projectName);
        result.put("status", response.status());
        result.put("response", response.body());
        return result;
    }

    public Map<String, Object> stopRecording() throws IOException {
        DeviceResponse response = executePost(stopUrl, new byte[0], false, "stop");

        Map<String, Object> result = new LinkedHashMap<>();
        result.put("success", true);
        result.put("status", response.status());
        result.put("response", response.body());
        return result;
    }

    private DeviceResponse executePost(String url, byte[] body, boolean jsonBody, String action) throws IOException {
        HttpURLConnection connection = (HttpURLConnection) new URL(url).openConnection(Proxy.NO_PROXY);
        connection.setConnectTimeout(timeoutMs);
        connection.setReadTimeout(timeoutMs);
        connection.setRequestMethod("POST");
        connection.setDoOutput(true);
        connection.setRequestProperty("accept", "application/json");
        if (jsonBody) {
            connection.setRequestProperty("Content-Type", "application/json");
        }
        connection.setFixedLengthStreamingMode(body.length);

        try (OutputStream out = connection.getOutputStream()) {
            out.write(body);
        }

        int status = connection.getResponseCode();
        String responseBody = readResponseBody(connection);

        if (status < 200 || status >= 300) {
            throw new IOException("Lidar " + action + " request failed, status=" + status + ", body=" + responseBody);
        }

        return new DeviceResponse(status, responseBody);
    }

    private String readResponseBody(HttpURLConnection connection) throws IOException {
        var stream = connection.getResponseCode() >= 400 ? connection.getErrorStream() : connection.getInputStream();
        if (stream == null) {
            return "";
        }
        try (stream) {
            return new String(stream.readAllBytes(), StandardCharsets.UTF_8);
        }
    }

    private record DeviceResponse(int status, String body) {
    }
}
