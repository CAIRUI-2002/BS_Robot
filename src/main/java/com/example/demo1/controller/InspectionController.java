package com.example.demo1.controller;

import com.example.demo1.model.ImageUploadRequest;
import com.example.demo1.model.ImageUploadResponse;
import com.example.demo1.model.InspectionResultDetail;
import com.example.demo1.model.InspectionResultRequest;
import com.example.demo1.model.InspectionResultSummary;
import com.example.demo1.service.InspectionResultService;
import org.springframework.beans.factory.annotation.Value;
import org.springframework.jdbc.core.JdbcTemplate;
import org.springframework.web.bind.annotation.*;

import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.time.LocalDate;
import java.time.format.DateTimeFormatter;
import java.util.Base64;
import java.util.List;
import java.util.Map;

@RestController
@RequestMapping("/api/inspection")
public class InspectionController {

    @Value("${inspection.storage-dir}")
    private String storageDir;

    @Value("${inspection.public-prefix}")
    private String publicPrefix;

    private final InspectionResultService inspectionResultService;
    private final JdbcTemplate jdbcTemplate;

    public InspectionController(InspectionResultService inspectionResultService,
                                JdbcTemplate jdbcTemplate) {
        this.inspectionResultService = inspectionResultService;
        this.jdbcTemplate = jdbcTemplate;
    }

    /**
     * 上传破损点实景抓拍图片。
     * 前端传 base64，后端保存成真实图片文件，返回 URL。
     */
    @PostMapping("/images/upload-base64")
    public ImageUploadResponse uploadBase64Image(@RequestBody ImageUploadRequest req) throws Exception {
        if (req.getFaultId() == null || req.getFaultId().isBlank()) {
            throw new IllegalArgumentException("faultId 不能为空");
        }

        if (req.getDirection() == null || req.getDirection().isBlank()) {
            throw new IllegalArgumentException("direction 不能为空");
        }

        if (req.getImageBase64() == null || req.getImageBase64().isBlank()) {
            throw new IllegalArgumentException("imageBase64 不能为空");
        }

        String base64 = req.getImageBase64();
        String ext = "jpg";

        if (base64.startsWith("data:image/png")) {
            ext = "png";
        }

        int commaIndex = base64.indexOf(",");
        if (commaIndex >= 0) {
            base64 = base64.substring(commaIndex + 1);
        }

        byte[] bytes = Base64.getDecoder().decode(base64);

        String dateDir = LocalDate.now().format(DateTimeFormatter.BASIC_ISO_DATE);

        Path dir = Paths.get(storageDir, "images", dateDir);
        Files.createDirectories(dir);

        String safeFaultId = req.getFaultId().replaceAll("[^a-zA-Z0-9_\\-]", "_");
        String safeDirection = req.getDirection().replaceAll("[^a-zA-Z0-9_\\-]", "_");

        String fileName = safeFaultId + "_" + safeDirection + "_" + System.currentTimeMillis() + "." + ext;

        Path target = dir.resolve(fileName);
        Files.write(target, bytes);

        String url = publicPrefix + "/images/" + dateDir + "/" + fileName;

        String fileType = "image_" + safeDirection;

        jdbcTemplate.update("""
            INSERT INTO inspection_file(
                related_fault_id,
                file_type,
                file_url,
                local_path,
                upload_device,
                file_size,
                remark
            ) VALUES (?, ?, ?, ?, ?, ?, ?)
        """,
                req.getFaultId(),
                fileType,
                url,
                target.toString(),
                "robot_dog",
                bytes.length,
                "破损点实景抓拍"
        );

        return new ImageUploadResponse(url);
    }

    /**
     * 保存一次巡检结果。
     */
    @PostMapping("/results")
    public Map<String, Object> saveInspectionResult(@RequestBody InspectionResultRequest req) throws Exception {
        Long id = inspectionResultService.saveInspectionResult(req);
        return Map.of(
                "id", id,
                "message", "saved"
        );
    }

    /**
     * 查询最近 5 次巡检历史。
     */
    @GetMapping("/results")
    public List<InspectionResultSummary> listInspectionResults(
            @RequestParam(defaultValue = "5") int limit) {
        return inspectionResultService.listRecent(limit);
    }

    /**
     * 查询某一次巡检详情，用于前端回显黄色电缆路径、红色破损点和图片 URL。
     */
    @GetMapping("/results/{id}")
    public InspectionResultDetail getInspectionResult(@PathVariable Long id) {
        return inspectionResultService.getDetail(id);
    }
}