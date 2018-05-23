package org.firstinspires.ftc.teamcode.vv7797.opmode.navigation;

import org.firstinspires.ftc.teamcode.vv7797.opmode.util.OpModeUtil.Plate;
import org.firstinspires.ftc.teamcode.vv7797.opmode.util.OpModeUtil.TeamColor;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;

public final class ArenaMap {
    private final Plate PLATE;
    private final TeamColor TEAM_COLOR;

    public static double ARENA_WIDTH = 12*12;
    public static double ARENA_HEIGHT = 12*12;
    public static double COLUMN_WIDTH = 7.5;
    public static double COLUMN_DEPTH = 4;
    public static double GLYPH_WIDTH = 6;
    public static double STONE_WIDTH = 23.5;
    public static double ROBOT_WIDTH = 18;
    public static double ROBOT_HEIGHT = 18;
    public static double INSERTION_RANGE = 9;
    public static double LATCH_PADDING = 2;

    public static Point PLATE_RED_BACK = new Point(10 * 12, 8 * 12);
    public static Point PLATE_RED_FRONT = new Point(10 * 12, 2 * 12);
    public static Point PLATE_BLUE_BACK = new Point(2 * 12, 8 * 12);
    public static Point PLATE_BLUE_FRONT = new Point(2 * 12, 2 * 12);

    public static Point FISHING_RED_BACK = new Point(0, 0);
    public static Point FISHING_RED_FRONT = new Point(8 * 12, 5 * 12);
    public static Point FISHING_BLUE_BACK = new Point(0, 0);
    public static Point FISHING_BLUE_FRONT = new Point(4 * 12, 5 * 12);

    public static double[] FRONT_RED_BOUNCE_DISTANCES = { 27, 20, 13 };
    public static double[] FRONT_BLUE_BOUNCE_DISTANCES = { 12, 19, 26 };
    public static double[] FRONT_ROUGH_DISTANCES = { 22, 15, 8 };

    public static Point[] COLUMNS_RED_BACK = {
        new Point(8 * 12, ARENA_HEIGHT - COLUMN_DEPTH),
        new Point(8 * 12 + COLUMN_WIDTH, ARENA_HEIGHT - COLUMN_DEPTH),
        new Point(8 * 12 + COLUMN_WIDTH * 2, ARENA_HEIGHT - COLUMN_DEPTH)
    };
    public static Point[] COLUMNS_RED_FRONT = {
        new Point(ARENA_WIDTH - COLUMN_DEPTH, 4 * 12 + COLUMN_WIDTH * 2),
        new Point(ARENA_WIDTH - COLUMN_DEPTH, 4 * 12 + COLUMN_WIDTH),
        new Point(ARENA_WIDTH - COLUMN_DEPTH, 4 * 12)
    };
    public static Point[] COLUMNS_BLUE_BACK = {
        new Point(2 * 12, ARENA_HEIGHT - COLUMN_DEPTH),
        new Point(2 * 12 + COLUMN_WIDTH, ARENA_HEIGHT - COLUMN_DEPTH),
        new Point(2 * 12 + COLUMN_WIDTH * 2, ARENA_HEIGHT - COLUMN_DEPTH)
    };
    public static Point[] COLUMNS_BLUE_FRONT = {
        new Point(COLUMN_DEPTH, 4 * 12),
        new Point(COLUMN_DEPTH, 4 * 12 + COLUMN_WIDTH),
        new Point(COLUMN_DEPTH, 4 * 12 + COLUMN_WIDTH * 2)
    };

    public static Point[] LATCHES_RED_BACK = {
        new Point(COLUMNS_RED_BACK[0].X + COLUMN_WIDTH / 2 + LATCH_PADDING, COLUMNS_RED_BACK[0].Y - INSERTION_RANGE-ROBOT_WIDTH / 2),
        new Point(COLUMNS_RED_BACK[1].X + COLUMN_WIDTH / 2 + LATCH_PADDING, COLUMNS_RED_BACK[1].Y - INSERTION_RANGE-ROBOT_WIDTH / 2),
        new Point(COLUMNS_RED_BACK[2].X + COLUMN_WIDTH / 2 + LATCH_PADDING, COLUMNS_RED_BACK[2].Y - INSERTION_RANGE-ROBOT_WIDTH / 2)
    };
    public static Point[] LATCHES_RED_FRONT = {
        new Point(COLUMNS_RED_FRONT[0].X - INSERTION_RANGE-ROBOT_WIDTH / 2, COLUMNS_RED_FRONT[0].Y + COLUMN_WIDTH / 2 - LATCH_PADDING),
        new Point(COLUMNS_RED_FRONT[1].X - INSERTION_RANGE-ROBOT_WIDTH / 2, COLUMNS_RED_FRONT[1].Y + COLUMN_WIDTH / 2 - LATCH_PADDING),
        new Point(COLUMNS_RED_FRONT[2].X - INSERTION_RANGE-ROBOT_WIDTH / 2, COLUMNS_RED_FRONT[2].Y + COLUMN_WIDTH / 2 - LATCH_PADDING)
    };
    public static Point[] LATCHES_BLUE_BACK = {
        new Point(COLUMNS_BLUE_BACK[0].X + COLUMN_WIDTH / 2 + LATCH_PADDING, COLUMNS_BLUE_BACK[0].Y - INSERTION_RANGE-ROBOT_WIDTH / 2),
        new Point(COLUMNS_BLUE_BACK[1].X + COLUMN_WIDTH / 2 + LATCH_PADDING, COLUMNS_BLUE_BACK[1].Y - INSERTION_RANGE-ROBOT_WIDTH / 2),
        new Point(COLUMNS_BLUE_BACK[2].X + COLUMN_WIDTH / 2 + LATCH_PADDING, COLUMNS_BLUE_BACK[2].Y - INSERTION_RANGE-ROBOT_WIDTH / 2)
    };
    public static Point[] LATCHES_BLUE_FRONT = {
        new Point(COLUMNS_BLUE_FRONT[0].X + INSERTION_RANGE+ROBOT_WIDTH / 2, COLUMNS_BLUE_FRONT[0].Y + COLUMN_WIDTH / 2 + LATCH_PADDING),
        new Point(COLUMNS_BLUE_FRONT[1].X + INSERTION_RANGE+ROBOT_WIDTH / 2, COLUMNS_BLUE_FRONT[1].Y + COLUMN_WIDTH / 2 + LATCH_PADDING),
        new Point(COLUMNS_BLUE_FRONT[2].X + INSERTION_RANGE+ROBOT_WIDTH / 2, COLUMNS_BLUE_FRONT[2].Y + COLUMN_WIDTH / 2 + LATCH_PADDING)
    };

    public static Point[] INSERTIONS_RED_BACK = {
            new Point(COLUMNS_RED_BACK[0].X + COLUMN_WIDTH / 2, COLUMNS_RED_BACK[0].Y - INSERTION_RANGE-ROBOT_WIDTH / 2),
            new Point(COLUMNS_RED_BACK[1].X + COLUMN_WIDTH / 2, COLUMNS_RED_BACK[1].Y - INSERTION_RANGE-ROBOT_WIDTH / 2),
            new Point(COLUMNS_RED_BACK[2].X + COLUMN_WIDTH / 2, COLUMNS_RED_BACK[2].Y - INSERTION_RANGE-ROBOT_WIDTH / 2)
    };
    public static Point[] INSERTIONS_RED_FRONT = {
            new Point(COLUMNS_RED_FRONT[0].X - INSERTION_RANGE-ROBOT_WIDTH / 2, COLUMNS_RED_FRONT[0].Y + COLUMN_WIDTH / 2),
            new Point(COLUMNS_RED_FRONT[1].X - INSERTION_RANGE-ROBOT_WIDTH / 2, COLUMNS_RED_FRONT[1].Y + COLUMN_WIDTH / 2),
            new Point(COLUMNS_RED_FRONT[2].X - INSERTION_RANGE-ROBOT_WIDTH / 2, COLUMNS_RED_FRONT[2].Y + COLUMN_WIDTH / 2)
    };
    public static Point[] INSERTIONS_BLUE_BACK = {
            new Point(COLUMNS_BLUE_BACK[0].X + COLUMN_WIDTH / 2, COLUMNS_BLUE_BACK[0].Y - INSERTION_RANGE-ROBOT_WIDTH / 2),
            new Point(COLUMNS_BLUE_BACK[1].X + COLUMN_WIDTH / 2, COLUMNS_BLUE_BACK[1].Y - INSERTION_RANGE-ROBOT_WIDTH / 2),
            new Point(COLUMNS_BLUE_BACK[2].X + COLUMN_WIDTH / 2, COLUMNS_BLUE_BACK[2].Y - INSERTION_RANGE-ROBOT_WIDTH / 2)
    };
    public static Point[] INSERTIONS_BLUE_FRONT = {
            new Point(COLUMNS_BLUE_FRONT[0].X + INSERTION_RANGE+ROBOT_WIDTH / 2, COLUMNS_BLUE_FRONT[0].Y + COLUMN_WIDTH / 2),
            new Point(COLUMNS_BLUE_FRONT[1].X + INSERTION_RANGE+ROBOT_WIDTH / 2, COLUMNS_BLUE_FRONT[1].Y + COLUMN_WIDTH / 2),
            new Point(COLUMNS_BLUE_FRONT[2].X + INSERTION_RANGE+ROBOT_WIDTH / 2, COLUMNS_BLUE_FRONT[2].Y + COLUMN_WIDTH / 2)
    };



    /**
     * @param plate Robot's plate
     * @param teamColor Robot's team color
     */
    public ArenaMap(Plate plate, TeamColor teamColor) {
        PLATE = plate;
        TEAM_COLOR = teamColor;
    }



    /**
     * @return Starting position on field given balancing stone and team color
     */
    public Point getStartingPosition() {
        switch (PLATE) {
            case BACK:
                if (TEAM_COLOR == TeamColor.RED)
                    return PLATE_RED_BACK;
                else
                    return PLATE_BLUE_BACK;

            case FRONT:
                if (TEAM_COLOR == TeamColor.RED)
                    return PLATE_RED_FRONT;
                else
                    return PLATE_BLUE_FRONT;

            default: return null;
        }
    }



    /**
     * @param mark Target column
     * @return Ideal latch position for alignment algorithm
     */
    public Point getColumnLatchPosition(RelicRecoveryVuMark mark) {
        // Select column set
        Point[] set = null;

        switch (PLATE) {
            case BACK:
                if (TEAM_COLOR == TeamColor.RED)
                    set = LATCHES_RED_BACK;
                else
                    set = LATCHES_BLUE_BACK;
            break;

            case FRONT:
                if (TEAM_COLOR == TeamColor.RED)
                    set = LATCHES_RED_FRONT;
                else
                    set = LATCHES_BLUE_FRONT;
            break;
        }

        // Select individual column coordinates
        switch (mark) {
            case LEFT: return set[0];
            case CENTER: return set[1];
            case RIGHT: return set[2];
            default: return  (PLATE == Plate.FRONT ? set[1] : (TEAM_COLOR == TeamColor.RED ? set[2] : set[0]));
        }
    }



    /**
     * @param mark Target column
     * @return Ideal glyph delivery position for some column
     */
    public Point getColumnInsertionPosition(RelicRecoveryVuMark mark) {
        // Select column set
        Point[] set = null;

        switch (PLATE) {
            case BACK:
                if (TEAM_COLOR == TeamColor.RED)
                    set = INSERTIONS_RED_BACK;
                else
                    set = INSERTIONS_BLUE_BACK;
                break;

            case FRONT:
                if (TEAM_COLOR == TeamColor.RED)
                    set = INSERTIONS_RED_FRONT;
                else
                    set = INSERTIONS_BLUE_FRONT;
                break;
        }

        // Select individual column coordinates
        switch (mark) {
            case LEFT: return set[0];
            case CENTER: return set[1];
            case RIGHT: return set[2];
            default: return  (PLATE == Plate.FRONT ? set[1] : (TEAM_COLOR == TeamColor.RED ? set[2] : set[0]));
        }
    }



    /**
     * @return Position from which to ram glyph pit
     */
    public Point getFishingPosition() {
        switch (PLATE) {
            case BACK:
                if (TEAM_COLOR == TeamColor.RED)
                    return FISHING_RED_BACK;
                else
                    return FISHING_BLUE_BACK;

            case FRONT:
                if (TEAM_COLOR == TeamColor.RED)
                    return FISHING_RED_FRONT;
                else
                    return FISHING_BLUE_FRONT;

            default: return new Point(-1, -1);
        }
    }
}
