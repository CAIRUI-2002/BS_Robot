package com.example.demo1.gpr.model;

import lombok.Data;

import java.util.List;

/**
 * 对应 MATLAB 代码开头那一大堆参数，作为后端可配置项。
 * 不传时用默认值，前端也可以通过 JSON 覆盖。
 */
@Data
public class GprConfig {

    // 扫描 / 数据参数
    private int ns = 512;              // 垂直采样点数
    private int headerBytes = 1024;    // DZT 头长度
    private double dtNs = 0.1;         // 采样间隔 ns
    private double dxM = 0.020;        // 道间距 m

    // 深度标尺速度
    private double vDepthMPerNs = 0.10;

    // 预处理
    private double scaleFactor = 1.0 / 256.0;
    private boolean depthGainOn = true;
    private double depthGainStrength = 1.8;

    // Z 字几何
    private double phiCableDeg = 0.0;
    private double thetaDeg = 45.0;
    private double startX = 0.0;
    private double startY = 0.0;

    // 方向控制
    private boolean useManualDirs = true;
    private List<Integer> dirsManual;    // 可为 null
    private boolean flipLREveryOther = true;
    private List<Boolean> flipLRFlags;   // 可为 null

    // 速度搜索与熵参数
    private double vSearchMin = 0.04;
    private double vSearchMax = 0.09;
    private int fibIters = 12;
    private int entropyBins = 256;
}
