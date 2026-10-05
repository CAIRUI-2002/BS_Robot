package com.example.demo1.gpr.service;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.assertNull;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.Random;
import java.util.List;

import org.junit.jupiter.api.Test;

import com.example.demo1.gpr.model.GprDetection;

class GprApexLocatorTest {

    @Test
    void findsApexInNoisyHyperbolaWithHorizontalReflection() {
        double[][] scan = syntheticScan(180, 230, 109, 44, 0.009, true);
        GprDetection box = new GprDetection(0, "cable", 0.9, 44, 35, 176, 115);

        GprApexLocator.Apex apex = GprApexLocator.locate(scan, box);

        assertNotNull(apex);
        assertEquals(109, apex.col(), 5);
        assertEquals(44, apex.row(), 5);
        assertEquals("bilateral", apex.method());
        assertWithinPreferredArea(apex, box);
    }

    @Test
    void tracesLeftWingWhenRightWingIsMissing() {
        double[][] scan = singleWingScan(-1);
        GprDetection box = new GprDetection(0, "cable", 0.9, 44, 35, 176, 115);

        GprApexLocator.Apex apex = GprApexLocator.locate(scan, box);

        assertNotNull(apex);
        assertEquals("singleWing", apex.method());
        assertEquals(109, apex.col(), 7);
        assertEquals(44, apex.row(), 7);
        assertWithinPreferredArea(apex, box);
    }

    @Test
    void tracesRightWingWhenLeftWingIsMissing() {
        double[][] scan = singleWingScan(1);
        GprDetection box = new GprDetection(0, "cable", 0.9, 44, 35, 176, 115);

        GprApexLocator.Apex apex = GprApexLocator.locate(scan, box);

        assertNotNull(apex);
        assertEquals("singleWing", apex.method());
        assertEquals(109, apex.col(), 7);
        assertEquals(44, apex.row(), 7);
        assertWithinPreferredArea(apex, box);
    }

    @Test
    void toleratesAWeakOppositeWingAndAShortGapInTheClearWing() {
        double[][] scan = singleWingScan(-1, 0.18, true);
        GprDetection box = new GprDetection(0, "cable", 0.9, 44, 35, 176, 115);

        GprApexLocator.Apex apex = GprApexLocator.locate(scan, box);

        assertNotNull(apex);
        assertEquals(109, apex.col(), 8);
        assertEquals(44, apex.row(), 8);
    }

    @Test
    void singleWingApexCanBeOffCenterWithinThePreferredArea() {
        double[][] scan = singleWingScan(1, 0.0, false, 82, 59);
        GprDetection box = new GprDetection(0, "cable", 0.9, 39, 51, 157, 139);

        GprApexLocator.Apex apex = GprApexLocator.locate(scan, box);

        assertNotNull(apex);
        assertEquals("singleWing", apex.method());
        assertEquals(82, apex.col(), 8);
        assertEquals(59, apex.row(), 8);
        assertWithinPreferredArea(apex, box);
    }

    @Test
    void findsApexWhenItIsNotAtBoxCenter() {
        double[][] scan = syntheticScan(180, 230, 82, 59, 0.008, false);
        GprDetection box = new GprDetection(0, "cable", 0.9, 39, 51, 157, 139);

        GprApexLocator.Apex apex = GprApexLocator.locate(scan, box);

        assertNotNull(apex);
        assertEquals(82, apex.col(), 5);
        assertEquals(59, apex.row(), 5);
        assertWithinPreferredArea(apex, box);
    }

    @Test
    void addsApexCoordinatesToYoloResult() {
        double[][] scan = syntheticScan(180, 230, 109, 44, 0.009, false);
        GprDetection box = new GprDetection(0, "cable", 0.9, 44, 35, 176, 115);

        GprApexLocator.locateAll(scan, List.of(box));

        assertNotNull(box.getApexCol());
        assertNotNull(box.getApexRow());
        assertNotNull(box.getApexScore());
        assertEquals("bilateral", box.getApexMethod());
        assertEquals(109, box.getApexCol(), 5);
        assertEquals(44, box.getApexRow(), 5);
    }

    @Test
    void doesNotInventApexForHorizontalLayer() {
        double[][] scan = new double[180][230];
        for (int row = 0; row < scan.length; row++) {
            double layer = 2 * ricker(row - 70, 3);
            for (int col = 0; col < scan[row].length; col++) scan[row][col] = layer;
        }
        GprDetection box = new GprDetection(0, "cable", 0.9, 40, 35, 170, 115);

        assertNull(GprApexLocator.locate(scan, box));
    }

    @Test
    void doesNotInventApexForNoise() {
        double[][] scan = new double[180][230];
        Random random = new Random(77);
        for (int row = 0; row < scan.length; row++) {
            for (int col = 0; col < scan[row].length; col++) {
                scan[row][col] = random.nextGaussian();
            }
        }
        GprDetection box = new GprDetection(0, "cable", 0.9, 40, 35, 170, 115);

        assertNull(GprApexLocator.locate(scan, box));
    }

    @Test
    void staysInUpperHalfWhenADeeperArcIsStronger() {
        double[][] scan = syntheticScan(200, 230, 110, 76, 0.008, false);
        addArc(scan, 110, 130, 0.008, 3.5);
        GprDetection box = new GprDetection(0, "cable", 0.9, 45, 50, 175, 160);

        GprApexLocator.Apex apex = GprApexLocator.locate(scan, box);

        assertNotNull(apex);
        assertEquals(110, apex.col(), 6);
        assertEquals(76, apex.row(), 6);
        assertWithinPreferredArea(apex, box);
    }

    @Test
    void doesNotSelectAnArcWhoseApexIsBelowTheBoxMidline() {
        double[][] scan = syntheticScan(200, 230, 110, 130, 0.008, false);
        GprDetection box = new GprDetection(0, "cable", 0.9, 45, 50, 175, 160);

        assertNull(GprApexLocator.locate(scan, box));
    }

    @Test
    void ignoresACompetingArcAlongTheTopBorder() {
        double[][] scan = syntheticScan(200, 230, 110, 82, 0.008, false);
        addArc(scan, 110, 50, 0.008, 2.8);
        GprDetection box = new GprDetection(0, "cable", 0.9, 45, 50, 175, 160);

        GprApexLocator.Apex apex = GprApexLocator.locate(scan, box);

        assertNotNull(apex);
        assertEquals(82, apex.row(), 6);
        assertWithinPreferredArea(apex, box);
    }

    private static double[][] syntheticScan(int rows, int cols, double apexCol,
                                            double apexRow, double curvature, boolean layer) {
        double[][] scan = new double[rows][cols];
        Random random = new Random(42);
        for (int row = 0; row < rows; row++) {
            for (int col = 0; col < cols; col++) {
                double ridgeRow = apexRow + curvature * Math.pow(col - apexCol, 2);
                scan[row][col] = 2.0 * ricker(row - ridgeRow, 2.5)
                        + (layer ? 1.6 * ricker(row - 20, 3) : 0)
                        + random.nextGaussian() * 0.035;
            }
        }
        return scan;
    }

    private static double[][] singleWingScan(int direction) {
        return singleWingScan(direction, 0.0, false);
    }

    private static double[][] singleWingScan(int direction, double weakAmplitude,
                                             boolean shortGap) {
        return singleWingScan(direction, weakAmplitude, shortGap, 109, 44);
    }

    private static double[][] singleWingScan(int direction, double weakAmplitude,
                                             boolean shortGap, double apexCol, double apexRow) {
        double[][] scan = new double[180][230];
        Random random = new Random(43 + direction);
        for (int row = 0; row < scan.length; row++) {
            for (int col = 0; col < scan[row].length; col++) {
                double offset = col - apexCol;
                double amplitude = offset * direction >= -2 ? 2.0 : weakAmplitude;
                if (shortGap && col >= 75 && col <= 79) amplitude = 0;
                double ridgeRow = apexRow + 0.009 * offset * offset;
                scan[row][col] = amplitude * ricker(row - ridgeRow, 2.5)
                        + 1.6 * ricker(row - 20, 3)
                        + random.nextGaussian() * 0.035;
            }
        }
        return scan;
    }

    private static double ricker(double offset, double sigma) {
        double value = offset / sigma;
        return (1 - value * value) * Math.exp(-value * value / 2);
    }

    private static void assertWithinPreferredArea(GprApexLocator.Apex apex, GprDetection box) {
        double width = box.getX2() - box.getX1();
        double height = box.getY2() - box.getY1();
        assertTrue(apex.col() >= box.getX1() + width * 0.25);
        assertTrue(apex.col() <= box.getX1() + width * 0.75);
        assertTrue(apex.row() >= box.getY1() + height * 0.05);
        assertTrue(apex.row() <= box.getY1() + height * 0.50);
    }

    private static void addArc(double[][] scan, double apexCol, double apexRow,
                               double curvature, double amplitude) {
        for (int row = 0; row < scan.length; row++) {
            for (int col = 0; col < scan[row].length; col++) {
                double ridgeRow = apexRow + curvature * Math.pow(col - apexCol, 2);
                scan[row][col] += amplitude * ricker(row - ridgeRow, 2.5);
            }
        }
    }
}
