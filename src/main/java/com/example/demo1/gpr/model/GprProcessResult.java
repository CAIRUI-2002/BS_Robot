package com.example.demo1.gpr.model;

import lombok.Data;

import java.util.List;

/**
 * 一次处理（一个目录）返回的总结果。
 */
@Data
public class GprProcessResult {

    private List<BScanResult> segments;   // 每个 .dzt 文件对应一段
    private double[] vBestStats;          // [min, median, max]

    // 整条 Z 字路径（折线），方便前端绘制
    private double[] pathX;
    private double[] pathY;
}
