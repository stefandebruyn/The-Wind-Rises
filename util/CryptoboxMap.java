package org.firstinspires.ftc.teamcode.vv7797.opmode.util;

import java.lang.reflect.Array;
import java.util.Arrays;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;

public final class CryptoboxMap {
    private final boolean[][] box = new boolean[4][3];



    /**
     * Log a glyph delivery
     *
     * @param column Column
     * @param glyphCount Number of glyphs delivered
     */
    public void deliver(RelicRecoveryVuMark column, int glyphCount) {
        // Find lowest free position in column
        int columnIndex = OpModeUtil.getColumnOrdinal(column) - 1;
        int deliveryPosition = -1;

        try {
            for (int i = box.length - 1; i >= 0; i--)
                if (!box[i][columnIndex]) {
                    deliveryPosition = i;
                    break;
                }
        } catch (ArrayIndexOutOfBoundsException e) {}

        // Log the delivery
        try {
            for (int i = deliveryPosition; i > deliveryPosition - glyphCount; i--)
                box[i][columnIndex] = true;
        } catch (ArrayIndexOutOfBoundsException e) {}
    }



    /**
     * @param column Column
     * @return Whether or not there are glyphs logged in a column
     */
    public boolean isColumnEmpty(RelicRecoveryVuMark column) {
        final int columnIndex = OpModeUtil.getColumnOrdinal(column) - 1;

        for (int i = 0; i < box.length; i++)
            if (box[i][columnIndex])
                return false;

        return true;
    }



    /**
     * Get the next column to deliver to
     *
     * @param currentColumn Current target column
     */
    public RelicRecoveryVuMark getNextColumn(RelicRecoveryVuMark currentColumn) {
        if (currentColumn == RelicRecoveryVuMark.CENTER)
            return (isColumnEmpty(RelicRecoveryVuMark.LEFT) ? RelicRecoveryVuMark.LEFT : RelicRecoveryVuMark.RIGHT);
        else if (currentColumn == RelicRecoveryVuMark.LEFT)
            return (isColumnEmpty(RelicRecoveryVuMark.CENTER) ? RelicRecoveryVuMark.CENTER : RelicRecoveryVuMark.RIGHT);
        else
            return (isColumnEmpty(RelicRecoveryVuMark.CENTER) ? RelicRecoveryVuMark.CENTER : RelicRecoveryVuMark.LEFT);
    }



    /**
     * @return deepToString() of the map
     */
    @Override public String toString() { return Arrays.deepToString(box); }
}
