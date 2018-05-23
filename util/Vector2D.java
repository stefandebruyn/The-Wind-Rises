package org.firstinspires.ftc.teamcode.vv7797.opmode.util;

import java.util.Locale;

public class Vector2D {

    private double dir, mag, x, y;



    /**
     * @param x Horizontal component
     * @param y Vertical component
     */
    public Vector2D(double x, double y) {
        this.x = x;
        this.y = y;

        dir = Math.atan(y/x);
        mag = Math.sqrt(x * x + y * y);
    }


    /**
     * @return Horizontal component
     */
    public double getX() { return x; }



    /**
     * @return Vertical component
     */
    public double getY() { return y; }



    /**
     * @return Direction (radians, [-pi/2, pi/2])
     */
    public double getDirection() { return dir; }



    /**
     * @return Magnitude
     */
    public double getMagnitude() { return mag; }


    /**
     * @param other Second operand
     * @return Sum of vectors
     */
    public Vector2D add(Vector2D other) { return new Vector2D(x + other.getX(), y + other.getY()); }



    /**
     * @param vectors Operands
     * @return Sum of vectors
     */
    public static Vector2D add(Vector2D... vectors) {
        Vector2D sum = new Vector2D(0, 0);

        for (Vector2D vector : vectors)
            sum = sum.add(vector);

        return sum;
    };



    @Override public String toString() { return String.format(Locale.getDefault(), "<%.4f, %.4f>", x, y); }
}
