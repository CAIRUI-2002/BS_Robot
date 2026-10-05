package com.example.demo1.gpr.service;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertNull;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.assertTrue;
import static org.mockito.ArgumentMatchers.any;
import static org.mockito.Mockito.mock;
import static org.mockito.Mockito.verify;
import static org.mockito.Mockito.when;

import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.List;

import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.io.TempDir;

import com.example.demo1.gpr.model.GprConfig;
import com.example.demo1.gpr.model.GprProcessResult;

class GprProcessingServiceMigrationTest {

    @TempDir
    Path tempDir;

    @Test
    void migrationIsDisabledByDefaultAndRawDetectionStillRuns() throws Exception {
        writeSmallDzt(tempDir.resolve("1.dzt"), 4, 4);

        GprYoloDetectionService detector = mock(GprYoloDetectionService.class);
        when(detector.detect(any(double[][].class))).thenReturn(List.of());
        GprProcessingService service = new GprProcessingService(detector);

        GprConfig config = new GprConfig();
        config.setNs(4);
        config.setHeaderBytes(0);
        config.setDepthGainOn(false);

        GprProcessResult result = service.processFolder(tempDir.toString(), config);

        assertFalse(config.isMigrationEnabled());
        assertFalse(result.isMigrationEnabled());
        assertEquals(1, result.getSegments().size());
        assertNull(result.getSegments().get(0).getMigratedData());
        verify(detector).detect(any(double[][].class));
    }

    @Test
    void migrationCanStillBeEnabledExplicitly() throws Exception {
        writeSmallDzt(tempDir.resolve("1.dzt"), 4, 4);

        GprYoloDetectionService detector = mock(GprYoloDetectionService.class);
        when(detector.detect(any(double[][].class))).thenReturn(List.of());
        GprProcessingService service = new GprProcessingService(detector);

        GprConfig config = new GprConfig();
        config.setNs(4);
        config.setHeaderBytes(0);
        config.setDepthGainOn(false);
        config.setMigrationEnabled(true);
        config.setAutoVSearch(false);

        GprProcessResult result = service.processFolder(tempDir.toString(), config);

        assertTrue(result.isMigrationEnabled());
        assertNotNull(result.getSegments().get(0).getMigratedData());
        assertEquals(4, result.getSegments().get(0).getMigratedData().length);
    }

    private static void writeSmallDzt(Path path, int samples, int traces) throws Exception {
        ByteBuffer buffer = ByteBuffer.allocate(samples * traces * Short.BYTES)
                .order(ByteOrder.LITTLE_ENDIAN);
        for (int trace = 0; trace < traces; trace++) {
            for (int sample = 0; sample < samples; sample++) {
                buffer.putShort((short) (1000 + trace * 10 + sample));
            }
        }
        Files.write(path, buffer.array());
    }
}
