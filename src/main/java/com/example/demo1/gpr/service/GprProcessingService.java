package com.example.demo1.gpr.service;

import java.io.IOException;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Comparator;
import java.util.List;
import java.util.stream.Collectors;

import org.springframework.stereotype.Service;

import com.example.demo1.gpr.model.BScanResult;
import com.example.demo1.gpr.model.GprConfig;
import com.example.demo1.gpr.model.GprProcessResult;
import com.example.demo1.gpr.model.SegmentGeometry;
import com.example.demo1.gpr.service.FftUtils.Complex;

/**
 * 将 MATLAB 中 GPT_dzt_10GHz_fk_3D_shagnzhi 的主要计算逻辑
 * 迁移到 Spring Boot 后端的 Service。
 */
@Service
public class GprProcessingService {

    /**
     * 处理指定目录下的所有 .dzt 文件
     */
    public GprProcessResult processFolder(String folderPath, GprConfig cfg) throws IOException {

        Path dir = Paths.get(folderPath);
        if (!Files.isDirectory(dir)) {
            throw new IllegalArgumentException("目录不存在: " + folderPath);
        }

        // 找到所有 .dzt 文件并排序
        List<Path> dztFiles = Files.list(dir)
                .filter(p -> p.toString().toLowerCase().endsWith(".dzt"))
                .sorted(Comparator.comparing(p -> p.getFileName().toString()))
                .collect(Collectors.toList());

        // === 新增：定义要截取的多段区间 ===
        // 注意：这里假设你在 GprConfig.java 中添加了 getTraceRanges() 方法。
        // 如果没有，你可以直接在这里写死这 5 段，例如：
        List<int[]> ranges = Arrays.asList(
            new int[]{4340, 4630},
            new int[]{4970, 5260},
            new int[]{22090, 22380},
            new int[]{22730, 23020},
            new int[]{23130, 23420}
        );

        // N 现在代表“总段数”，而不是“文件数”
        int N = dztFiles.size() * ranges.size();
        if (N == 0) {
            throw new IllegalArgumentException("该目录下没有 .dzt 文件或未指定截取段");
        }

        // 方向 & 翻转标记 (依据总段数生成，确保 5 段能自动 Z 字形排布)
        int[] dirs = buildDirs(N, cfg);
        boolean[] flipLR = buildFlipFlags(N, cfg);

        List<BScanResult> results = new ArrayList<>();

        double[] segPhi = new double[N];
        double[] segLen = new double[N];
        double[][] segStartXY = new double[N + 1][2];
        segStartXY[0][0] = cfg.getStartX();
        segStartXY[0][1] = cfg.getStartY();

        double phiCable = Math.toRadians(cfg.getPhiCableDeg());
        double theta = Math.toRadians(cfg.getThetaDeg());
        double vDepth = cfg.getVDepthMPerNs();

        double[] vOptList = new double[N];

        System.out.println("共发现 " + dztFiles.size() + " 个 .dzt 文件，计划切分为 " + N + " 段处理……");

        // === 核心修改：改为双重循环 ===
        int globalIndex = 0; // 全局索引，用于对应 dirs 和 flipLR 数组

        for (Path file : dztFiles) {
            for (int[] range : ranges) {
                int startTrace = range[0];
                int endTrace = range[1];

                // === 1. 读取 DZT 局部数据 ===
                DztReadResult read = readDztUint16(file, cfg, startTrace, endTrace);
                double[][] data = read.data; // [ns x nr]
                int nz = data.length;
                int nu = data[0].length;

                // 水平翻转 (使用全局索引)
                if (flipLR[globalIndex]) {
                    data = flipLeftRight(data);
                }

                // === 2. 去直流 ===
                removeDcPerTrace(data);

                // === 3. 深度增益 ===
                if (cfg.isDepthGainOn()) {
                    applyDepthGain(data, cfg.getDepthGainStrength());
                }

                // === 4. 斐波那契搜索，最小化偏移图像熵 ===
                FibResult fib = fibonacciMinEntropy(
                        data,
                        cfg.getDtNs(),
                        cfg.getDxM(),
                        cfg.getVSearchMin(),
                        cfg.getVSearchMax(),
                        cfg.getFibIters(),
                        cfg.getEntropyBins()
                );
                double vBest = fib.vBest;
                double hBest = fib.hBest;
                vOptList[globalIndex] = vBest;

                // === 5. 在最优速度处做一次 Stolt F-K 偏移 ===
                double[][] migrated = stoltFkMigration(
                        data,
                        cfg.getDtNs(),
                        cfg.getDxM(),
                        vBest
                );

                // === 6. Z 字几何计算 ===
                double phi = phiCable + dirs[globalIndex] * theta;
                segPhi[globalIndex] = phi;
                double length = (nu - 1) * cfg.getDxM();
                segLen[globalIndex] = length;

                if (globalIndex < N) {
                    double dxSeg = length * Math.cos(phi);
                    double dySeg = length * Math.sin(phi);
                    segStartXY[globalIndex + 1][0] = segStartXY[globalIndex][0] + dxSeg;
                    segStartXY[globalIndex + 1][1] = segStartXY[globalIndex][1] + dySeg;
                }

                // === 7. 计算该段 u / z 轴 ===
                double[] uAxis = new double[nu];
                for (int c = 0; c < nu; c++) {
                    uAxis[c] = c * cfg.getDxM();
                }
                double[] zAxis = new double[nz];
                for (int r = 0; r < nz; r++) {
                    double tNs = r * cfg.getDtNs();
                    zAxis[r] = 0.5 * vDepth * tNs;
                }

                SegmentGeometry geo = new SegmentGeometry();
                geo.setPhiRad(phi);
                geo.setLength(length);
                geo.setStartX(segStartXY[globalIndex][0]);
                geo.setStartY(segStartXY[globalIndex][1]);
                geo.setUAxis(uAxis);
                geo.setZAxis(zAxis);

                BScanResult segResult = new BScanResult();
                // 给文件名加上范围后缀，方便前端区分是哪一段
                segResult.setFileName(file.getFileName().toString() + "_" + startTrace + "-" + endTrace);
                segResult.setNz(nz);
                segResult.setNu(nu);
                segResult.setRawData(data);
                segResult.setMigratedData(migrated);
                segResult.setVBest(vBest);
                segResult.setHBest(hBest);
                segResult.setGeometry(geo);

                results.add(segResult);

                System.out.printf(
                        "  [%2d/%2d] %s (道号:%d-%d) | traces=%d dir=%+d flip=%b | v* = %.5f m/ns%n",
                        globalIndex + 1, N, file.getFileName(), startTrace, endTrace, nu, 
                        dirs[globalIndex], flipLR[globalIndex], vBest
                );

                globalIndex++; // 全局索引递增
            }
        }

        // === 8. 折线路径（Z 字路径） ===
        double[] pathX = new double[N + 1];
        double[] pathY = new double[N + 1];
        for (int i = 0; i <= N; i++) {
            pathX[i] = segStartXY[i][0];
            pathY[i] = segStartXY[i][1];
        }

        // === 9. 统计最优速度 min / median / max ===
        Arrays.sort(vOptList);
        double vMin = vOptList[0];
        double vMax = vOptList[N - 1];
        double vMed = vOptList[N / 2];

        GprProcessResult out = new GprProcessResult();
        out.setSegments(results);
        out.setPathX(pathX);
        out.setPathY(pathY);
        out.setVBestStats(new double[]{vMin, vMed, vMax});

        System.out.println("处理完成。");
        return out;
    }

    // ========================= 方向 / 翻转控制 =============================

    /**
     * 生成每段的方向数组 dirs[i]，取值为 +1 或 -1。
     *
     * 约定（和后面 phi = phiCable + dir * theta 保持一致）：
     *   dir = +1  → 第 i 段朝 “右前方”
     *   dir = -1  → 第 i 段朝 “左前方”
     *
     * 默认（useManualDirs=true 且 dirsManual 为空时）：
     *   dirs = [+1, -1, +1, -1, ...]
     *   → 第 1 段右前，第 2 段左前，依次交替
     */
    private int[] buildDirs(int N, GprConfig cfg) {
        int[] dirs = new int[N];
        List<Integer> manual = cfg.getDirsManual();

        if (cfg.isUseManualDirs()) {
            if (manual == null || manual.size() != N) {
                // 默认：+1, -1, +1, -1, ...  → 第 1 段右前，第 2 段左前
                for (int i = 0; i < N; i++) {
                    dirs[i] = (i % 2 == 0) ? +1 : -1;
                }
            } else {
                // 使用用户提供的方向，只保留符号（>=0 看成 +1，<0 看成 -1）
                for (int i = 0; i < N; i++) {
                    int d = manual.get(i);
                    dirs[i] = (d >= 0) ? +1 : -1;
                }
            }
        } else {
            // 不使用手动方向时，可以使用另一种交替规则（这里反过来）
            for (int i = 0; i < N; i++) {
                dirs[i] = (i % 2 == 0) ? -1 : +1;
            }
        }
        return dirs;
    }


    /**
     * 生成每段是否左右翻转的数组 flags[i]。
     *
     * 默认（flipLRFlags 为空、flipLREveryOther=true 时）：
     *   段号 1,2,3,4,... 中：
     *     第 2、4、6... 段翻转 → false, true, false, true, ...
     */
    private boolean[] buildFlipFlags(int N, GprConfig cfg) {
        boolean[] flags = new boolean[N];
        List<Boolean> manual = cfg.getFlipLRFlags();

        if (manual != null && manual.size() == N) {
            for (int i = 0; i < N; i++) {
                flags[i] = Boolean.TRUE.equals(manual.get(i));
            }
        } else {
            if (cfg.isFlipLREveryOther()) {
                // 段号从 1 开始：2、4、6... 段翻转
                for (int i = 0; i < N; i++) {
                    int segIndex = i + 1;          // 1,2,3,...
                    flags[i] = (segIndex % 2 == 0); // 第 2、4、6... 段 true
                }
            } else {
                Arrays.fill(flags, false);         // 全部不翻转
            }
        }
        return flags;
    }


    // ========================= 读取 DZT (支持 uint16 格式) =============================

    private static class DztReadResult {
        double[][] data;
        int nr;
    }

    /**
     * 读取 DZT 文件（uint16 格式，支持局部读取）
     * * @param file DZT 文件路径
     * @param cfg 配置参数
     * @param startTrace 起始道数
     * @param endTrace 结束道数
     * @return 读取结果
     */
    private DztReadResult readDztUint16(Path file, GprConfig cfg, int startTrace, int endTrace) throws IOException {

        byte[] all = Files.readAllBytes(file);
        if (all.length <= cfg.getHeaderBytes()) {
            throw new IllegalArgumentException("文件过小或 headerBytes 设置有误: " + file);
        }

        int dataBytes = all.length - cfg.getHeaderBytes();
        int bytesPerSample = cfg.getBytesPerSample();
        int totalTraces = dataBytes / (cfg.getNs() * bytesPerSample);
        
        if (totalTraces <= 0) {
            throw new IllegalArgumentException("推算不到合理的道数: " + file);
        }

        // 校正范围（防止前端传的范围超出文件实际道数）
        if (startTrace < 1) startTrace = 1;
        if (endTrace > totalTraces) endTrace = totalTraces;
        if (startTrace > endTrace) {
            throw new IllegalArgumentException("起始道数不能大于结束道数！文件: " + file);
        }
        
        int nr = endTrace - startTrace + 1;
        
        // 计算起始偏移量
        int startOffset = cfg.getHeaderBytes() + (startTrace - 1) * cfg.getNs() * bytesPerSample;
        int readLength = nr * cfg.getNs() * bytesPerSample;
        
        if (startOffset + readLength > all.length) {
            throw new IllegalArgumentException("读取范围超出文件大小: " + file);
        }

        ByteBuffer buf = ByteBuffer.wrap(all, startOffset, readLength);
        buf.order(ByteOrder.LITTLE_ENDIAN);

        double[][] A = new double[cfg.getNs()][nr];
        
        // 根据字节数选择读取方式
        if (bytesPerSample == 2) {
            // uint16 格式 (新设备)
            for (int c = 0; c < nr; c++) {
                for (int r = 0; r < cfg.getNs(); r++) {
                    int unsignedVal = buf.getShort() & 0xFFFF;
                    A[r][c] = unsignedVal * cfg.getScaleFactor();
                }
            }
        } else if (bytesPerSample == 4) {
            // int32 格式 (旧设备兼容)
            for (int c = 0; c < nr; c++) {
                for (int r = 0; r < cfg.getNs(); r++) {
                    A[r][c] = buf.getInt() * cfg.getScaleFactor();
                }
            }
        } else {
            throw new IllegalArgumentException("不支持的字节数: " + bytesPerSample);
        }

        DztReadResult res = new DztReadResult();
        res.data = A;
        res.nr = nr;
        return res;
    }

    // ========================= 基础矩阵操作 =============================

    private double[][] flipLeftRight(double[][] data) {
        int rows = data.length;
        int cols = data[0].length;
        double[][] out = new double[rows][cols];
        for (int r = 0; r < rows; r++) {
            for (int c = 0; c < cols; c++) {
                out[r][c] = data[r][cols - 1 - c];
            }
        }
        return out;
    }

    private void removeDcPerTrace(double[][] data) {
        int rows = data.length;
        int cols = data[0].length;
        for (int c = 0; c < cols; c++) {
            double sum = 0;
            for (int r = 0; r < rows; r++) {
                sum += data[r][c];
            }
            double mean = sum / rows;
            for (int r = 0; r < rows; r++) {
                data[r][c] -= mean;
            }
        }
    }

    private void applyDepthGain(double[][] data, double strength) {
        int rows = data.length;
        int cols = data[0].length;
        for (int r = 0; r < rows; r++) {
            double gain = 1 + 0.0006 * Math.pow(r, strength);
            for (int c = 0; c < cols; c++) {
                data[r][c] *= gain;
            }
        }
    }

    // ========================= 斐波那契搜索最小熵 =============================

    private static class FibResult {
        double vBest;
        double hBest;
    }

    private FibResult fibonacciMinEntropy(
            double[][] data,
            double dtNs,
            double dxM,
            double vmin,
            double vmax,
            int Nfib,
            int nbins
    ) {
        if (Nfib < 3) Nfib = 3;

        long[] F = new long[Nfib + 2];
        F[0] = 1;
        F[1] = 1;
        for (int i = 2; i < Nfib + 2; i++) {
            F[i] = F[i - 1] + F[i - 2];
        }

        double a = vmin;
        double b = vmax;
        int k = Nfib;

        double x1 = a + (double) F[k - 2] / F[k] * (b - a);
        double x2 = a + (double) F[k - 1] / F[k] * (b - a);

        double H1 = costEntropy(x1, data, dtNs, dxM, nbins);
        double H2 = costEntropy(x2, data, dtNs, dxM, nbins);

        for (k = Nfib; k >= 3; k--) {
            if (H1 > H2) {
                a = x1;
                x1 = x2;
                H1 = H2;
                x2 = a + (double) F[k - 1] / F[k] * (b - a);
                H2 = costEntropy(x2, data, dtNs, dxM, nbins);
            } else {
                b = x2;
                x2 = x1;
                H2 = H1;
                x1 = a + (double) F[k - 2] / F[k] * (b - a);
                H1 = costEntropy(x1, data, dtNs, dxM, nbins);
            }
        }

        FibResult out = new FibResult();
        if (H1 <= H2) {
            out.vBest = x1;
            out.hBest = H1;
        } else {
            out.vBest = x2;
            out.hBest = H2;
        }
        return out;
    }

    /**
     * 熵代价函数：
     *  - 在指定速度下做一次 Stolt F-K 偏移
     *  - 取幅值 |mig| → 归一化到 [0,1]
     *  - 统计直方图并计算香农熵
     */
    private double costEntropy(
            double vMPerNs, double[][] data,
            double dtNs, double dxM, int nbins
    ) {
        double[][] mig = stoltFkMigration(data, dtNs, dxM, vMPerNs);
        int rows = mig.length;
        int cols = mig[0].length;

        double min = Double.POSITIVE_INFINITY;
        double max = Double.NEGATIVE_INFINITY;
        double[] flatAbs = new double[rows * cols];
        int idx = 0;
        for (int r = 0; r < rows; r++) {
            for (int c = 0; c < cols; c++) {
                double val = Math.abs(mig[r][c]);
                flatAbs[idx++] = val;
                if (val < min) min = val;
                if (val > max) max = val;
            }
        }

        if (max <= min) {
            return 0.0;
        }

        // 归一化到 [0,1]
        for (int i = 0; i < flatAbs.length; i++) {
            flatAbs[i] = (flatAbs[i] - min) / (max - min);
        }

        // 统计直方图 & 熵
        double[] bins = new double[nbins];
        for (double v : flatAbs) {
            int b = (int) Math.floor(v * nbins);
            if (b == nbins) b = nbins - 1;
            bins[b] += 1.0;
        }

        double total = flatAbs.length;
        double H = 0.0;
        for (int i = 0; i < nbins; i++) {
            if (bins[i] > 0) {
                double p = bins[i] / total;
                H -= p * (Math.log(p) / Math.log(2)); // log2
            }
        }
        return H;
    }

    // ========================= Stolt F-K 偏移 =============================

    private double[][] stoltFkMigration(
            double[][] data,
            double dtNs,
            double dxM,
            double vMPerNs
    ) {
        int ns = data.length;
        int nr = data[0].length;

        double dt = dtNs * 1e-9;      // s
        double v = vMPerNs * 1e9;     // m/s

        // 1. 时间方向 Hann 窗
        double[] winT = new double[ns];
        for (int i = 0; i < ns; i++) {
            winT[i] = 0.5 * (1 - Math.cos(2 * Math.PI * i / (ns - 1)));
        }
        double[][] dataW = new double[ns][nr];
        for (int r = 0; r < ns; r++) {
            for (int c = 0; c < nr; c++) {
                dataW[r][c] = data[r][c] * winT[r];
            }
        }

        // 2. 扩展到 2 的幂次
        int Nt = nextPow2(2 * ns);
        int Nx = nextPow2(2 * nr);

        // 3. 填充到 Nt x Nx 复数矩阵
        Complex[][] pad = new Complex[Nt][Nx];
        for (int r = 0; r < Nt; r++) {
            for (int c = 0; c < Nx; c++) {
                double val = 0.0;
                if (r < ns && c < nr) {
                    val = dataW[r][c];
                }
                pad[r][c] = new Complex(val, 0.0);
            }
        }

        // 4. 2D FFT，然后在 x 方向 fftshift
        Complex[][] D = FftUtils.fft2D(pad);
        Complex[][] D_fk = FftUtils.fftshiftX(D);

        // 5. 频率 / 波数轴
        double df = 1.0 / (Nt * dt);
        int Nw = Nt / 2 + 1;
        double[] w = new double[Nw];
        for (int i = 0; i < Nw; i++) {
            w[i] = 2 * Math.PI * i * df;
        }

        double dkx = 2 * Math.PI / (Nx * dxM);
        double[] kx = new double[Nx];
        for (int i = 0; i < Nx; i++) {
            kx[i] = (i - Nx / 2) * dkx;
        }

        // 6. 正频部分 + Stolt 映射
        Complex[][] Qpos = new Complex[Nw][Nx];

        for (int ix = 0; ix < Nx; ix++) {

            // 当前列 Dpos(:, ix)
            Complex[] col = new Complex[Nw];
            for (int iw = 0; iw < Nw; iw++) {
                col[iw] = D_fk[iw][ix];
            }

            // 映射后的频率 ws
            double[] ws = new double[Nw];
            for (int iw = 0; iw < Nw; iw++) {
                double w0 = w[iw];
                ws[iw] = Math.sqrt(w0 * w0 + (v * v * kx[ix] * kx[ix]) / 4.0);
            }

            // 线性插值 Pout = interp1(w, col, ws)
            Complex[] pout = FftUtils.interp1Complex(w, col, ws);

            // scale = w ./ max(ws, eps)
            for (int iw = 0; iw < Nw; iw++) {
                double w0 = w[iw];
                double wsVal = ws[iw];
                double scale = (wsVal > 0) ? (w0 / wsVal) : 0.0;
                if (iw == 0) scale = 0.0; // DC 分量置 0

                Qpos[iw][ix] = pout[iw].scale(scale);
            }
        }

        // 7. 补全负频，得到 Nt x Nx 的 Qfull
        Complex[][] Qfull = new Complex[Nt][Nx];

        // 正频：0..Nw-1
        for (int iw = 0; iw < Nw; iw++) {
            for (int ix = 0; ix < Nx; ix++) {
                Qfull[iw][ix] = Qpos[iw][ix];
            }
        }

        // 负频（共轭对称）
        if (Nt % 2 == 0) {
            // Nt 偶数：idx_neg = (Nw-1):-1:2
            int outRow = Nw;
            for (int iw = Nw - 1; iw >= 2 && outRow < Nt; iw--) {
                for (int ix = 0; ix < Nx; ix++) {
                    Qfull[outRow][ix] = Qpos[iw][ix].conj();
                }
                outRow++;
            }
        } else {
            // Nt 奇数：idx_neg = Nw:-1:2
            int outRow = Nw;
            for (int iw = Nw; iw >= 2 && outRow < Nt; iw--) {
                for (int ix = 0; ix < Nx; ix++) {
                    Qfull[outRow][ix] = Qpos[iw][ix].conj();
                }
                outRow++;
            }
        }

        // 8. x 方向 ifftshift，再 2D IFFT 回到 (t,x)
        Complex[][] Q_unshift = FftUtils.ifftshiftX(Qfull);
        Complex[][] q_tx = FftUtils.ifft2D(Q_unshift);

        // 9. 取实部并裁剪回 ns x nr
        double[][] migrated = new double[ns][nr];
        for (int r = 0; r < ns; r++) {
            for (int c = 0; c < nr; c++) {
                migrated[r][c] = q_tx[r][c].re;
            }
        }

        return migrated;
    }

    private int nextPow2(int n) {
        int p = 1;
        while (p < n) p <<= 1;
        return p;
    }
}
