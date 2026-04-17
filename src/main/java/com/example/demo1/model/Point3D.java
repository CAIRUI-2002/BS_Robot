package com.example.demo1.model;

public class Point3D {
    private double x;
    private double y;
    private double z;
    private int r;
    private int g;
    private int b;

    public Point3D(double x, double y, double z, int r, int g, int b) {
        this.x = x;
        this.y = y;
        this.z = z;
        this.r = r;
        this.g = g;
        this.b = b;
    }

    // Getters
    public double getX() { return x; }
    public double getY() { return y; }
    public double getZ() { return z; }
    public int getR() { return r; }
    public int getG() { return g; }
    public int getB() { return b; }
}