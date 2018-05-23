package org.firstinspires.ftc.teamcode.vv7797.opmode.util;

public class SimpleColor {
    private final int R, G, B;



    /**
     * @param r Red
     * @param g Green
     * @param b Blue
     */
    public SimpleColor(int r, int g, int b) {
        R = r;
        G = g;
        B = b;
    }



    /**
     * @return Red
     */
    public int red() { return R; }



    /**
     * @return Green
     */
    public int green() { return G; }



    /**
     * @return Blue
     */
    public int blue() { return B; }



    /**
     * Get a color's relative similarity to this one
     *
     * @param other Comparison color
     * @return Color deviation (arbitrary)
     */
    public double getDeviation(SimpleColor other) {
        final int rErr = Math.abs(other.red() - R);
        final int gErr = Math.abs(other.green() - G);
        final int bErr = Math.abs(other.blue() - B);

        return rErr + gErr + bErr;
    }
}
