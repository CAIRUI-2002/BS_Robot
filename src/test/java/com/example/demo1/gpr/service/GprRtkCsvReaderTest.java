package com.example.demo1.gpr.service;

import static org.junit.jupiter.api.Assertions.assertArrayEquals;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;

import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.io.TempDir;

class GprRtkCsvReaderTest {

    @TempDir
    Path tempDir;

    @Test
    void readsExplicitDztTraceMappingInBscanOrder() throws Exception {
        Path dzt = tempDir.resolve("scan_001.dzt");
        Files.write(dzt, new byte[]{0});
        Files.writeString(
                tempDir.resolve("scan_001.csv"),
                "\ufeffbscan_column,dzt_trace_index_1based,latitude_deg,longitude_deg,fix_description\n" +
                        "2,3,30.000002,114.000002,\"RTK固定解\"\n" +
                        "1,2,30.000001,114.000001,\"RTK固定解\"\n" +
                        "3,4,30.000003,114.000003,\"RTK固定解\"\n",
                StandardCharsets.UTF_8
        );

        GprRtkCsvReader.RtkTracePath path = GprRtkCsvReader.read(dzt, 4).orElseThrow();

        assertArrayEquals(new int[]{2, 3, 4}, path.traceIndices1Based);
        assertArrayEquals(
                new double[]{30.000001, 30.000002, 30.000003},
                path.latitudeDeg,
                1e-12
        );
        assertEquals(3, path.sourceRowCount);
        assertEquals(3, path.validCoordinateCount);
        assertEquals("scan_001.csv", path.csvPath.getFileName().toString());
    }

    @Test
    void infersSkippedPlaceholderTraceAndInterpolatesMissingCoordinate() throws Exception {
        Path dzt = tempDir.resolve("scan_legacy.dzt");
        Files.write(dzt, new byte[]{0});
        Files.writeString(
                tempDir.resolve("scan_legacy.csv"),
                "latitude_deg,longitude_deg\n" +
                        "30.000000,114.000000\n" +
                        ",\n" +
                        "30.000002,114.000004\n",
                StandardCharsets.UTF_8
        );

        GprRtkCsvReader.RtkTracePath path = GprRtkCsvReader.read(dzt, 4).orElseThrow();

        assertArrayEquals(new int[]{2, 3, 4}, path.traceIndices1Based);
        assertEquals(30.000001, path.latitudeDeg[1], 1e-12);
        assertEquals(114.000002, path.longitudeDeg[1], 1e-12);
        assertEquals(2, path.validCoordinateCount);
        assertTrue(path.latitudeDeg.length == path.longitudeDeg.length);
    }

    @Test
    void spreadsLowRateRtkCoordinateRunsAcrossRadarTraces() throws Exception {
        Path dzt = tempDir.resolve("scan_plateau.dzt");
        Files.write(dzt, new byte[]{0});
        Files.writeString(
                tempDir.resolve("scan_plateau.csv"),
                "bscan_column,dzt_trace_index_1based,latitude_deg,longitude_deg\n" +
                        "1,2,30.000000,114.000000\n" +
                        "2,3,30.000000,114.000000\n" +
                        "3,4,30.000000,114.000000\n" +
                        "4,5,30.000003,114.000006\n" +
                        "5,6,30.000003,114.000006\n" +
                        "6,7,30.000003,114.000006\n",
                StandardCharsets.UTF_8
        );

        GprRtkCsvReader.RtkTracePath path = GprRtkCsvReader.read(dzt, 7).orElseThrow();

        assertEquals(30.000000, path.latitudeDeg[0], 1e-12);
        assertEquals(30.000003, path.latitudeDeg[5], 1e-12);
        for (int i = 1; i < path.latitudeDeg.length; i++) {
            assertTrue(path.latitudeDeg[i] > path.latitudeDeg[i - 1]);
            assertTrue(path.longitudeDeg[i] > path.longitudeDeg[i - 1]);
        }
    }

}
