package com.example.demo1.controller;

import com.example.demo1.gpr.model.GprConfig;
import com.example.demo1.gpr.model.GprProcessResult;
import com.example.demo1.gpr.service.GprProcessingService;
import org.springframework.beans.factory.annotation.Autowired;
import org.springframework.web.bind.annotation.*;

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
}
