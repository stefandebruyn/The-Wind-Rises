package org.firstinspires.ftc.teamcode.vv7797.opmode.util;

import java.util.HashMap;
import java.util.Map;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;

/**
 * A collection of various utilities used across multiple op modes
 */
public abstract class OpModeUtil {
    public enum TeamColor { RED, BLUE }
    public enum Plate { FRONT, BACK }
    public enum GlyphColor { GRAY, BROWN, AMBIGUOUS, NONE };

    private static Map<GlyphColor, SimpleColor> glyphColorProfiles = new HashMap<GlyphColor, SimpleColor>() {{
       put(GlyphColor.GRAY, new SimpleColor(60, 60, 50));
       put(GlyphColor.BROWN, new SimpleColor(40, 40, 30));
       put(GlyphColor.NONE, new SimpleColor(50, 50, 40));
    }};



    /**
     * Get the minimum difference between two angles
     *
     * @param a Angle measure one (degrees, signed)
     * @param b Angle measure two (degrees, signed)
     * @return The smallest difference between the two angles (e.g. angleDev(0, 360) => 0)
     */
    public static double angleDev(double a, double b) {
        double diff = b - a;
        diff += (diff > 180 ? -360 : (diff < -180 ? 360 : 0));
        return diff;
    }



    /**
     * Get the individual wheel rotation signs associated with some relative direction. Note: if a bad direction is specified
     * zeros will be returned
     *
     * @param dir Direction (unsigned, degrees, multiple of 45 on [0, 315])
     * @return Rotation signs [front left, front right, back left, back right]
     */
    public static int[] getMecanumWheelSigns(int dir) {
        switch (dir) {
            case   0: return new int[] { 1,  -1, -1,  1 };
            case  45: return new int[] {  1,  0,  0,  1 };
            case  90: return new int[] {  1,  1,  1,  1 };
            case 135: return new int[] {  0,  1,  1,  0 };
            case 180: return new int[] { -1,  1,  1, -1 };
            case 225: return new int[] { -1,  0,  0, -1 };
            case 270: return new int[] { -1, -1, -1, -1 };
            case 315: return new int[] {  0, -1, -1,  0 };
            default:  return new int[] {  0,  0,  0,  0 };
        }
    }



    /**
     * Convert centimeters to inches
     *
     * @param cm Centimeters
     * @return Equivalent distance in inches
     */
    public static double cmToIn(double cm) {
        return cm * 0.393701;
    }



    /**
     * Get the absolute angle relative to another absolute angle by some amount
     *
     * @param abs Absolute angle (degrees)
     * @param rel Relative angle (degrees)
     * @return Coterminal relative angle on [0.0, 360.0) (degrees)
     */
    public static double getAbsRelToAbs(double abs, double rel) {
        return (abs + rel - 90) % 360;
    }



    /**
     * Get the relative equivalent of an absolute angle given current absolute angle
     *
     * @param a Desired absolute on [0.0, 360.0) (degrees)
     * @param r Current absolute on [0.0, 360.0) (degrees)
     * @return Equivalent relative angle on [0.0, 360.0) (degrees)
     */
    public static double getRelEquOfAbs(double a, double r) { return (((360 + a - r) % 360) + 90) % 360; }



    /**
     * Constrain a degree angle to [0, 360)
     *
     * @param a Angle
     * @return Wrapped angle
     */
    public static double wrapAngleStandard(double a) {
        if (a >= 360)
            return a % 360;
        else if (a < 0)
            return 360 + a;
        else
            return a;
    }



    /**
     * Constrain a degree angle to [-180, 180)
     *
     * @param a Angle
     * @return Wrapped angle
     */
    public static double wrapAngleGyro(double a) {
        if (a >= 180)
            return a % 360;
        else if (a < -180)
            return 180 + a;
        else
            return a;
    }



    /**
     * Get the 1-based index associated with some pictogram
     *
     * @param column Column
     * @return Ordinal
     */
    public static int getColumnOrdinal(RelicRecoveryVuMark column) {
        switch (column) {
            case LEFT: return 1;
            case CENTER: return 2;
            case RIGHT: return 3;
            default: return 0;
        }
    }



    /**
     * Predict a glyph's color from color sensor data
     *
     * @param r Red
     * @param g Green
     * @param b Blue
     * @return Most likely glyph color
     */
    public static GlyphColor getGlyphColor(int r, int g, int b) {
        final SimpleColor sample = new SimpleColor(r, g, b);
        GlyphColor recordHolder = null;
        double record = Double.MAX_VALUE;

        for (GlyphColor glyph : glyphColorProfiles.keySet()) {
            double rating = glyphColorProfiles.get(glyph).getDeviation(sample);

            if (rating < record) {
                record = rating;
                recordHolder = glyph;
            }
        }

        return recordHolder;
    }



    /**
     * Convert linear velocity to angular velocity
     *
     * @param linearVelocity Linear velocity
     * @param radius Radius
     * @return Equivalent angular velocity
     */
    public static double linearToAngular(double linearVelocity, double radius) {
        final double radiansToLinear = (Math.PI * 2) / (Math.PI * radius * 2);

        return linearVelocity * radiansToLinear;
    }



    /**
     * Convert angular velocity to linear velocity
     *
     * @param angularVelocity Angular velocity
     * @param radius Radius
     * @return Equivalent linear velocity
     */
    public static double angularToLinear(double angularVelocity, double radius) {
        final double linearToRadians = (Math.PI * radius * 2) / (Math.PI * 2);

        return angularVelocity * linearToRadians;
    }
}
