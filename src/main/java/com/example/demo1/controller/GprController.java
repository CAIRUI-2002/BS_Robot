package com.example.demo1.controller;

import java.io.File;

import org.springframework.beans.factory.annotation.Autowired;
import org.springframework.core.io.FileSystemResource;
import org.springframework.core.io.Resource;
import org.springframework.http.HttpHeaders;
import org.springframework.http.MediaType;
import org.springframework.http.ResponseEntity;
import org.springframework.web.bind.annotation.GetMapping;
import org.springframework.web.bind.annotation.PostMapping;
import org.springframework.web.bind.annotation.RequestBody;
import org.springframework.web.bind.annotation.RequestMapping;
import org.springframework.web.bind.annotation.RequestParam;
import org.springframework.web.bind.annotation.RestController;

import com.example.demo1.gpr.model.GprConfig;
import com.example.demo1.gpr.model.GprProcessResult;
import com.example.demo1.gpr.service.GprProcessingService;

/**
 * 对前端暴露的接口：
 *   POST /api/gpr/process?folderPath=...
 * 返回处理后的所有数据（JSON）
 */
@RestController
@RequestMapping("/api/gpr")
public class GprController {

    @Autowired
    private GprProcessingService gprProcessingService;

    /**
     * 处理服务器本地某个目录下的全部 .dzt 文件
     *
     * 用法示例：
     *   POST http://localhost:8080/api/gpr/process?folderPath=D:/data/dzt
     *
     * body（可选 JSON）：
     *   {
     *     "ns": 512,
     *     "dtNs": 0.1,
     *     "dxM": 0.02,
     *     "vSearchMin": 0.04,
     *     "vSearchMax": 0.09
     *   }
     */
    @PostMapping("/process")
    public GprProcessResult processFolder(
            @RequestParam("folderPath") String folderPath,
            @RequestBody(required = false) GprConfig config
    ) throws Exception {
        if (config == null) {
            config = new GprConfig(); // 使用默认参数
        }
        return gprProcessingService.processFolder(folderPath, config);
    }
    /**
     * 读取绝对路径的点云文件并返回流给 Three.js PCDLoader
     * 用法示例： GET /api/gpr/pcd?filePath=/mnt/e/data/map.pcd
     */
    @GetMapping("/pcd")
    public ResponseEntity<Resource> getPcdFile(@RequestParam("filePath") String filePath) {
        File file = new File(filePath);
        if (!file.exists() || !file.isFile()) {
            return ResponseEntity.notFound().build();
        }
        
        Resource resource = new FileSystemResource(file);
        return ResponseEntity.ok()
                // 告诉浏览器这是一个附件流
                .header(HttpHeaders.CONTENT_DISPOSITION, "attachment; filename=\"" + file.getName() + "\"")
                .contentType(MediaType.APPLICATION_OCTET_STREAM)
                .body(resource);
    }
}
