package org.firstinspires.ftc.teamcode.vv7797.opmode.util;

public final class MatrixBuilder {
    private boolean checkArithmetic = false;



    public MatrixBuilder(boolean checkArithmetic) {
        this.checkArithmetic = checkArithmetic;
    }



    public boolean getCheckArithmetic() {
        return checkArithmetic;
    }



    public Matrix build(int rows, int cols) {
        return new Matrix(rows, cols);
    }



    public Matrix build(int rows, int cols, double... entries) {
        Matrix mat = new Matrix(rows, cols, entries);
        mat.checkArithmetic = checkArithmetic;

        return mat;
    }
}
