package com.example.demo1.gpr.model;

import lombok.Data;

import java.util.List;

/**
 * 一次处理（一个目录）返回的总结果。
 */
@Data
public class GprProcessResult {

    private List<BScanResult> segments;   // 每个 .dzt 文件对应一段
    private boolean migrationEnabled;     // 本次处理是否执行了偏移成像
    private double[] vBestStats;          // [min, median, max]；未偏移时为配置的固定偏移速度

    // 整条 Z 字路径（折线），方便前端绘制
    private double[] pathX;
    private double[] pathY;
}
