package com.example.demo1.gpr.service;

import java.io.IOException;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.Comparator;
import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.Locale;
import java.util.Map;
import java.util.Optional;
import java.util.Set;

/**
 * 读取与 DZT 同名的 CSV，并建立“B-scan 列 -> DZT 道号 -> RTK 经纬度”映射。
 */
final class GprRtkCsvReader {

    private GprRtkCsvReader() {
    }

    static Optional<RtkTracePath> read(Path dztPath, int totalDztTraces) throws IOException {
        Path csvPath = findSidecarCsv(dztPath);
        if (csvPath == null) return Optional.empty();

        List<String> lines = Files.readAllLines(csvPath, StandardCharsets.UTF_8);
        if (lines.size() < 2) return Optional.empty();

        List<String> headers = parseCsvLine(stripBom(lines.get(0)));
        Map<String, Integer> columns = new HashMap<>();
        for (int i = 0; i < headers.size(); i++) {
            columns.put(headers.get(i).trim().toLowerCase(Locale.ROOT), i);
        }

        Integer latColumn = columns.get("latitude_deg");
        Integer lonColumn = columns.get("longitude_deg");
        if (latColumn == null || lonColumn == null) return Optional.empty();

        Integer traceColumn = columns.get("dzt_trace_index_1based");
        Integer orderColumn = columns.get("bscan_column");
        List<RawRecord> records = new ArrayList<>();

        for (int lineIndex = 1; lineIndex < lines.size(); lineIndex++) {
            if (lines.get(lineIndex).isBlank()) continue;
            List<String> fields = parseCsvLine(lines.get(lineIndex));
            int traceIndex = traceColumn == null
                    ? Integer.MIN_VALUE
                    : parseInteger(getField(fields, traceColumn), Integer.MIN_VALUE);
            int order = orderColumn == null
                    ? records.size() + 1
                    : parseInteger(getField(fields, orderColumn), records.size() + 1);
            double latitude = parseDouble(getField(fields, latColumn));
            double longitude = parseDouble(getField(fields, lonColumn));
            records.add(new RawRecord(traceIndex, order, latitude, longitude));
        }

        if (records.size() < 2) return Optional.empty();

        // 旧版 CSV 没有显式 DZT 道号时，按行数推断：等长从第1道开始，少1行时
        // 从第2道开始（兼容本设备每个 DZT 的首道常值占位数据）。
        if (traceColumn == null) {
            int inferredStart = records.size() == totalDztTraces - 1 ? 2 : 1;
            for (int i = 0; i < records.size(); i++) {
                records.get(i).traceIndex1Based = inferredStart + i;
            }
        }

        records.sort(Comparator.comparingInt(record -> record.order));
        List<RawRecord> mapped = new ArrayList<>();
        Set<Integer> seenTraceIndices = new HashSet<>();
        for (RawRecord record : records) {
            if (record.traceIndex1Based < 1 || record.traceIndex1Based > totalDztTraces) continue;
            if (!seenTraceIndices.add(record.traceIndex1Based)) continue;
            mapped.add(record);
        }
        if (mapped.size() < 2) return Optional.empty();

        int validCoordinateCount = 0;
        for (RawRecord record : mapped) {
            if (isValidCoordinate(record.latitudeDeg, record.longitudeDeg)) validCoordinateCount++;
        }
        if (validCoordinateCount < 2) return Optional.empty();

        interpolateMissingCoordinates(mapped);
        if (!hasSpatialSpan(mapped)) return Optional.empty();
        interpolateCoordinateRunsToEveryTrace(mapped);

        int size = mapped.size();
        int[] traceIndices = new int[size];
        double[] latitude = new double[size];
        double[] longitude = new double[size];
        for (int i = 0; i < size; i++) {
            RawRecord record = mapped.get(i);
            traceIndices[i] = record.traceIndex1Based;
            latitude[i] = record.latitudeDeg;
            longitude[i] = record.longitudeDeg;
        }

        return Optional.of(new RtkTracePath(
                csvPath,
                traceIndices,
                latitude,
                longitude,
                records.size(),
                validCoordinateCount
        ));
    }

    private static Path findSidecarCsv(Path dztPath) throws IOException {
        String fileName = dztPath.getFileName().toString();
        int dot = fileName.lastIndexOf('.');
        String stem = dot >= 0 ? fileName.substring(0, dot) : fileName;
        Path exact = dztPath.resolveSibling(stem + ".csv");
        if (Files.isRegularFile(exact)) return exact;

        try (var stream = Files.list(dztPath.getParent())) {
            return stream
                    .filter(Files::isRegularFile)
                    .filter(path -> path.getFileName().toString().equalsIgnoreCase(stem + ".csv"))
                    .findFirst()
                    .orElse(null);
        }
    }

    private static void interpolateMissingCoordinates(List<RawRecord> records) {
        for (int i = 0; i < records.size(); i++) {
            RawRecord current = records.get(i);
            if (isValidCoordinate(current.latitudeDeg, current.longitudeDeg)) continue;

            int previous = i - 1;
            while (previous >= 0 && !isValidCoordinate(
                    records.get(previous).latitudeDeg,
                    records.get(previous).longitudeDeg
            )) previous--;

            int next = i + 1;
            while (next < records.size() && !isValidCoordinate(
                    records.get(next).latitudeDeg,
                    records.get(next).longitudeDeg
            )) next++;

            if (previous >= 0 && next < records.size()) {
                RawRecord a = records.get(previous);
                RawRecord b = records.get(next);
                double denominator = b.traceIndex1Based - a.traceIndex1Based;
                double fraction = Math.abs(denominator) > 1e-9
                        ? (current.traceIndex1Based - a.traceIndex1Based) / denominator
                        : (double) (i - previous) / (next - previous);
                fraction = Math.max(0.0, Math.min(1.0, fraction));
                current.latitudeDeg = a.latitudeDeg + fraction * (b.latitudeDeg - a.latitudeDeg);
                current.longitudeDeg = a.longitudeDeg + fraction * (b.longitudeDeg - a.longitudeDeg);
            } else if (previous >= 0) {
                current.latitudeDeg = records.get(previous).latitudeDeg;
                current.longitudeDeg = records.get(previous).longitudeDeg;
            } else if (next < records.size()) {
                current.latitudeDeg = records.get(next).latitudeDeg;
                current.longitudeDeg = records.get(next).longitudeDeg;
            }
        }
    }

    private static boolean hasSpatialSpan(List<RawRecord> records) {
        RawRecord first = records.get(0);
        for (int i = 1; i < records.size(); i++) {
            RawRecord record = records.get(i);
            if (Math.abs(record.latitudeDeg - first.latitudeDeg) > 1e-9 ||
                    Math.abs(record.longitudeDeg - first.longitudeDeg) > 1e-9) {
                return true;
            }
        }
        return false;
    }

    /**
     * RTK更新频率通常低于雷达道频，CSV中会出现多个相邻道共用同一坐标的台阶。
     * 以每个连续坐标段的中心作为RTK控制点，对全部雷达道做分段线性插值，既保留
     * 实测测线形状和端点，又避免三维网格出现大量零宽度退化三角形。
     */
    private static void interpolateCoordinateRunsToEveryTrace(List<RawRecord> records) {
        if (records.size() < 3) return;

        List<CoordinateAnchor> anchors = new ArrayList<>();
        anchors.add(new CoordinateAnchor(0, records.get(0).latitudeDeg, records.get(0).longitudeDeg));

        int runStart = 0;
        while (runStart < records.size()) {
            int runEnd = runStart;
            RawRecord first = records.get(runStart);
            while (runEnd + 1 < records.size()) {
                RawRecord next = records.get(runEnd + 1);
                if (!sameCoordinate(first, next)) break;
                runEnd++;
            }

            if (runStart > 0 && runEnd < records.size() - 1) {
                int center = (runStart + runEnd) / 2;
                anchors.add(new CoordinateAnchor(center, first.latitudeDeg, first.longitudeDeg));
            }
            runStart = runEnd + 1;
        }

        int lastIndex = records.size() - 1;
        RawRecord last = records.get(lastIndex);
        if (anchors.get(anchors.size() - 1).index != lastIndex) {
            anchors.add(new CoordinateAnchor(lastIndex, last.latitudeDeg, last.longitudeDeg));
        }
        if (anchors.size() < 2) return;

        int segment = 0;
        for (int i = 0; i < records.size(); i++) {
            while (segment + 1 < anchors.size() - 1 && i > anchors.get(segment + 1).index) {
                segment++;
            }
            CoordinateAnchor a = anchors.get(segment);
            CoordinateAnchor b = anchors.get(Math.min(segment + 1, anchors.size() - 1));
            double fraction = b.index == a.index ? 0.0 : (double) (i - a.index) / (b.index - a.index);
            fraction = Math.max(0.0, Math.min(1.0, fraction));
            records.get(i).latitudeDeg = a.latitudeDeg + fraction * (b.latitudeDeg - a.latitudeDeg);
            records.get(i).longitudeDeg = a.longitudeDeg + fraction * (b.longitudeDeg - a.longitudeDeg);
        }
    }

    private static boolean sameCoordinate(RawRecord a, RawRecord b) {
        return Math.abs(a.latitudeDeg - b.latitudeDeg) <= 1e-11 &&
                Math.abs(a.longitudeDeg - b.longitudeDeg) <= 1e-11;
    }

    private static boolean isValidCoordinate(double latitude, double longitude) {
        return Double.isFinite(latitude) && Double.isFinite(longitude) &&
                latitude >= -90.0 && latitude <= 90.0 &&
                longitude >= -180.0 && longitude <= 180.0 &&
                !(Math.abs(latitude) < 1e-12 && Math.abs(longitude) < 1e-12);
    }

    private static String getField(List<String> fields, int index) {
        return index >= 0 && index < fields.size() ? fields.get(index).trim() : "";
    }

    private static int parseInteger(String value, int fallback) {
        try {
            return Integer.parseInt(value.trim());
        } catch (RuntimeException ignored) {
            return fallback;
        }
    }

    private static double parseDouble(String value) {
        try {
            return Double.parseDouble(value.trim());
        } catch (RuntimeException ignored) {
            return Double.NaN;
        }
    }

    private static String stripBom(String value) {
        return value != null && !value.isEmpty() && value.charAt(0) == '\ufeff'
                ? value.substring(1)
                : value;
    }

    private static List<String> parseCsvLine(String line) {
        List<String> fields = new ArrayList<>();
        StringBuilder current = new StringBuilder();
        boolean quoted = false;
        for (int i = 0; i < line.length(); i++) {
            char ch = line.charAt(i);
            if (ch == '"') {
                if (quoted && i + 1 < line.length() && line.charAt(i + 1) == '"') {
                    current.append('"');
                    i++;
                } else {
                    quoted = !quoted;
                }
            } else if (ch == ',' && !quoted) {
                fields.add(current.toString());
                current.setLength(0);
            } else {
                current.append(ch);
            }
        }
        fields.add(current.toString());
        return fields;
    }

    static final class RtkTracePath {
        final Path csvPath;
        final int[] traceIndices1Based;
        final double[] latitudeDeg;
        final double[] longitudeDeg;
        final int sourceRowCount;
        final int validCoordinateCount;

        RtkTracePath(
                Path csvPath,
                int[] traceIndices1Based,
                double[] latitudeDeg,
                double[] longitudeDeg,
                int sourceRowCount,
                int validCoordinateCount
        ) {
            this.csvPath = csvPath;
            this.traceIndices1Based = traceIndices1Based;
            this.latitudeDeg = latitudeDeg;
            this.longitudeDeg = longitudeDeg;
            this.sourceRowCount = sourceRowCount;
            this.validCoordinateCount = validCoordinateCount;
        }

    }

    private static final class RawRecord {
        int traceIndex1Based;
        final int order;
        double latitudeDeg;
        double longitudeDeg;

        RawRecord(int traceIndex1Based, int order, double latitudeDeg, double longitudeDeg) {
            this.traceIndex1Based = traceIndex1Based;
            this.order = order;
            this.latitudeDeg = latitudeDeg;
            this.longitudeDeg = longitudeDeg;
        }
    }

    private record CoordinateAnchor(int index, double latitudeDeg, double longitudeDeg) {
    }
}
