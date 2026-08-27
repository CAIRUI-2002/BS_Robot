package com.example.demo1.gpr.model;

import java.util.List;

import lombok.Data;

/**
 * GPR 配置类
 * 对应 MATLAB 代码中的参数，作为后端可配置项。
 * 不传时用默认值，前端也可以通过 JSON 覆盖。
 * 
 * 更新日志：
 * - 2024-03-19: 适配新探地雷达设备（uint16格式)
 * - 新增介电常数参数，分离偏移速度和深度换算速度
 * - 支持局部数据读取
 */
@Data
public class GprConfig {

    // ==================== 数据读取参数 ====================
    private int ns = 512; // 垂直采样点数
    private int headerBytes = 1024; // DZT 头长度 (字节)
    private int bytesPerSample = 2; // 每个采样点的字节数 (uint16 = 2, int32 = 4)
    private double timeWindow = 50.0; // 测量时窗

    // ==================== 物理参数 ====================
    private double dxM = 0.010; // 道间距 - 新设备默认1cm
    private double epsilonR = 8.0; // 介电常数 (ε_r) - 新增
    private final double cLight = 0.3; // 光速 (m/ns)

    // ==================== 速度参数（分离偏移速度和深度换算速度）====================
    private double vMigMPerNs = 0.07; // 偏移聚焦速度 - 仅用于F-K算法
    private boolean autoVSearch = true; // 是否自动搜索最优偏移速度
    private double vSearchMin = 0.05; // 最小搜索速度
    private double vSearchMax = 0.20; // 最大搜索速度
    private int fibIters = 12; // 斐波那契迭代次数
    private int entropyBins = 256; // 熵计算bins数

    // ==================== 预处理参数 ====================
    private double scaleFactor = 1.0; // 数据缩放因子 (uint16通常不需要缩放)
    private boolean depthGainOn = true; // 默认启用温和的深度增益
    private double depthGainStrength = 1.0; // 增益强度系数，1.0 表示使用完整增益曲线
    private double depthGainCurveExponent = 0.35; // 基础深度增益曲线指数
    private double depthGainShallowEnd = 0.18; // 红框上边界：0%～18% 为浅层
    private double depthGainMiddleEnd = 0.72; // 红框下边界：18%～72% 为中层
    private double depthGainTransitionWidth = 0.04; // 分层边界平滑过渡宽度
    private double depthGainShallowScale = 0.03; // 浅层只保留极少增益
    private double depthGainMiddleScale = 1.30; // 中层增强基础增益增量
    private double depthGainDeepScale = 1.0; // 深层保持基础增益
    private double depthGainMax = 2.5; // 浅层/深层最大增益
    private double depthGainMiddleMax = 2.8; // 中层增强后的最大增益

    // ==================== Z 字几何参数 ====================
    private double phiCableDeg = 0.0; // 电缆初始角度 (度)
    private double thetaDeg = 60.0; // Z字转角 (度)
    private double startX = 0.0; // 起点X坐标
    private double startY = 0.0; // 起点Y坐标

    // ==================== 方向控制 ====================
    private boolean useManualDirs = true;
    private List<Integer> dirsManual; // 可为 null
    private boolean flipLREveryOther = true;
    private List<Boolean> flipLRFlags; // 可为 null

    // ==================== 计算属性 ====================

    /**
     * 根据介电常数计算深度换算速度
     * 公式: v = c / sqrt(ε_r)
     */
    public double getVDepthMPerNs() {
        return cLight / Math.sqrt(epsilonR);
    }

    /**
     * 计算采样间隔
     * dt = timeWindow / ns (ns)
     */
    public double getDtNs() {
        return timeWindow / ns;
    }
}
