package org.firstinspires.ftc.teamcode.vv7797.opmode.navigation;

public class Point {
    public final double X, Y;

    public Point(double x, double y) { X = x; Y = y; }

    @Override public String toString() { return "(" + X + ", " + Y + ")"; }
}
