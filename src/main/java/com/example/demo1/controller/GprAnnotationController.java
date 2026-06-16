package com.example.demo1.controller;

import com.fasterxml.jackson.databind.ObjectMapper;
import org.springframework.http.ResponseEntity;
import org.springframework.web.bind.annotation.DeleteMapping;
import org.springframework.web.bind.annotation.GetMapping;
import org.springframework.web.bind.annotation.PostMapping;
import org.springframework.web.bind.annotation.RequestBody;
import org.springframework.web.bind.annotation.RequestMapping;
import org.springframework.web.bind.annotation.RequestParam;
import org.springframework.web.bind.annotation.RestController;

import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.nio.file.StandardCopyOption;
import java.time.Instant;
import java.util.LinkedHashMap;
import java.util.Map;

@RestController
@RequestMapping("/api/gpr/annotation")
public class GprAnnotationController {

    private static final String ANNOTATION_FILE_NAME = "gpr_annotation_calibration.json";

    private final ObjectMapper objectMapper = new ObjectMapper();

    @GetMapping
    public ResponseEntity<Map<String, Object>> loadAnnotation(@RequestParam String folderPath) throws Exception {
        Path folder = resolveExistingFolder(folderPath, false);
        Path annotationFile = folder.resolve(ANNOTATION_FILE_NAME).normalize();

        Map<String, Object> result = new LinkedHashMap<>();
        result.put("exists", Files.exists(annotationFile));
        result.put("filePath", annotationFile.toString());
        result.put("fileName", ANNOTATION_FILE_NAME);

        if (!Files.exists(annotationFile)) {
            return ResponseEntity.ok(result);
        }

        String json = Files.readString(annotationFile, StandardCharsets.UTF_8);
        Object payload = objectMapper.readValue(json, Object.class);
        result.put("payload", payload);
        return ResponseEntity.ok(result);
    }

    @PostMapping
    public ResponseEntity<Map<String, Object>> saveAnnotation(@RequestBody Map<String, Object> body) throws Exception {
        Object folderPathObj = body.get("folderPath");
        Object payloadObj = body.get("payload");

        if (folderPathObj == null || String.valueOf(folderPathObj).isBlank()) {
            throw new IllegalArgumentException("folderPath 不能为空");
        }
        if (payloadObj == null) {
            throw new IllegalArgumentException("payload 不能为空");
        }

        Path folder = resolveExistingFolder(String.valueOf(folderPathObj), true);
        Path annotationFile = folder.resolve(ANNOTATION_FILE_NAME).normalize();
        Path tmpFile = folder.resolve(ANNOTATION_FILE_NAME + ".tmp").normalize();

        Map<String, Object> payload;
        if (payloadObj instanceof Map<?, ?> inputMap) {
            payload = new LinkedHashMap<>();
            inputMap.forEach((k, v) -> payload.put(String.valueOf(k), v));
        } else {
            payload = new LinkedHashMap<>();
            payload.put("data", payloadObj);
        }

        payload.put("serverSavedAt", Instant.now().toString());
        payload.put("annotationFileName", ANNOTATION_FILE_NAME);

        String json = objectMapper.writerWithDefaultPrettyPrinter().writeValueAsString(payload);
        Files.writeString(tmpFile, json, StandardCharsets.UTF_8);
        try {
            Files.move(tmpFile, annotationFile, StandardCopyOption.REPLACE_EXISTING, StandardCopyOption.ATOMIC_MOVE);
        } catch (Exception atomicMoveError) {
            Files.move(tmpFile, annotationFile, StandardCopyOption.REPLACE_EXISTING);
        }

        Map<String, Object> result = new LinkedHashMap<>();
        result.put("success", true);
        result.put("filePath", annotationFile.toString());
        result.put("fileName", ANNOTATION_FILE_NAME);
        return ResponseEntity.ok(result);
    }

    @DeleteMapping
    public ResponseEntity<Map<String, Object>> deleteAnnotation(@RequestParam String folderPath) throws Exception {
        Path folder = resolveExistingFolder(folderPath, false);
        Path annotationFile = folder.resolve(ANNOTATION_FILE_NAME).normalize();

        boolean deleted = false;
        if (Files.exists(annotationFile)) {
            Files.delete(annotationFile);
            deleted = true;
        }

        Map<String, Object> result = new LinkedHashMap<>();
        result.put("success", true);
        result.put("deleted", deleted);
        result.put("filePath", annotationFile.toString());
        result.put("fileName", ANNOTATION_FILE_NAME);
        return ResponseEntity.ok(result);
    }

    private Path resolveExistingFolder(String folderPath, boolean requireWritable) {
        if (folderPath == null || folderPath.isBlank()) {
            throw new IllegalArgumentException("GPR 数据文件夹路径不能为空");
        }

        Path folder = Paths.get(folderPath).toAbsolutePath().normalize();

        if (!Files.exists(folder)) {
            throw new IllegalArgumentException("GPR 数据文件夹不存在：" + folder);
        }
        if (!Files.isDirectory(folder)) {
            throw new IllegalArgumentException("GPR 数据路径不是文件夹：" + folder);
        }
        if (requireWritable && !Files.isWritable(folder)) {
            throw new IllegalArgumentException("GPR 数据文件夹不可写，请检查机器狗主机目录权限：" + folder);
        }

        return folder;
    }
}
