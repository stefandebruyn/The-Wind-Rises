package org.firstinspires.ftc.teamcode.vv7797.opmode.util;

/**
 * Encapsulates a two-dimensional matrix of reals. Supports basic arithmetic,
 * transposition, and string formatting with custom column width. All arithmetic
 * operations have the appropriate input checking; attempting illegal matrix
 * arithmetic will result in a RuntimeException (unless arithmetic checking is
 * specifically disabled). Other than this, no attempt is made at preventing
 * overflowing or nonreal mathematics.
 */

public class Matrix {
  public double[][] matrix;
  public int formatColumnWidth = 10;
  public boolean checkArithmetic = true;


  /**
   * Create an empty matrix
   *
   * @param rows Height
   * @param cols Length
   */
  public Matrix(int rows, int cols) {
    matrix = new double[rows][cols];
  }



  /**
   * Create a matrix with contents
   *
   * @param rows Height
   * @param cols Length
   * @param entries Contents
   */
  public Matrix(int rows, int cols, double... entries) {
    matrix = new double[rows][cols];
    fill(entries);
  }



  /**
   * Set the format width of columns for Matrix.toString(). Defaults to 10.
   *
   * @param w Desired width
   * @see Matrix.toString()
   */
  public void setFormatColumnWidth(int w) { formatColumnWidth = w; }



  /**
   * Get the number of rows in this matrix
   *
   * @return Row count
   */
  public int getRows() {
    return matrix.length;
  }



  /**
   * Get the number of columns in this matrix
   *
   * @return Column count
   */
  public int getCols() {
    return matrix[0].length;
  }



  /**
   * Get the value at some position in this matrix
   *
   * @param row Row
   * @param col Column
   * @return Value
   */
  public double get(int row, int col) {
    return matrix[row][col];
  }



    /**
     * Set the value at some position in this matrix
     *
     * @param row Row
     * @param col Column
     * @param val Value
     */
    public void set(int row, int col, double val) {
        matrix[row][col] = val;
    }



  /**
   * Fill the matrix with values. If the provided entries are not enough to fill the
   * matrix, the remaining positions go unchanged.
   *
   * @param entries Contents
   */
  public void fill(double... entries) {
    for (int row = 0; row < getRows(); row++)
      for (int col = 0; col < getCols(); col++)
        try {
          matrix[row][col] = entries[row * getCols() + col];
        } catch (IndexOutOfBoundsException e) {
          break;
        }
  }



  /**
   * Multiply this matrix by another. Operands remain unchanged.
   *
   * @param other Second operand
   * @return Product matrix
   * @throws RuntimeException If the two matrices are not eligible for multiplication
   */
  public Matrix multiply(Matrix other) throws RuntimeException {
    if (checkArithmetic && getCols() != other.getRows())
      throw new RuntimeException("Illegal multiplication on matrices of sizes [" + getRows() + ", " + getCols() + "] and [" + other.getRows() + ", " + other.getCols() + "]");

    Matrix product = new Matrix(getRows(), other.getCols());
    double[] entries = new double[getRows() * other.getCols()];

    for (int pRow = 0; pRow < product.getRows(); pRow++)
      for (int pCol = 0; pCol < product.getCols(); pCol++) {
        double entry = 0;
        for (int ind = 0; ind < getCols(); ind++)
          entry += matrix[pRow][ind] * other.get(ind, pCol);
        entries[pRow * product.getCols() + pCol] = entry;
      }

    product.fill(entries);
    return product;
  }



  /**
   * Divide this matrix by another. Operands remain unchanged.
   *
   * @param other Second operand
   * @return Quotient matrix
   * @throws RuntimeException If the two matrices are not eligible for division
   */
  public Matrix divide(Matrix other) throws RuntimeException {
    if (checkArithmetic && getCols() != other.getRows())
      throw new RuntimeException("Illegal division on matrices of sizes [" + getRows() + ", " + getCols() + "] and [" + other.getRows() + ", " + other.getCols() + "]");

    Matrix quotient = new Matrix(getRows(), getCols());
    double[] entries = new double[getRows() * getCols()];

    for (int row = 0; row < getRows(); row++)
      for (int col = 0; col < getCols(); col++)
        entries[row * getCols() + col] = matrix[row][col] / other.get(row, col);

    quotient.fill(entries);
    return quotient;
  }



  /**
   * Add to this matrix from another. Operands remain unchanged.
   *
   * @param other Second operand
   * @return Sum matrix
   * @throws RuntimeException If the two matrices are not eligible for addition
   */
  public Matrix add(Matrix other) {
    if (checkArithmetic && getCols() != other.getCols() || getRows() != other.getRows())
      throw new RuntimeException("Illegal addition on matrices of sizes [" + getRows() + ", " + getCols() + "] and [" + other.getRows() + ", " + other.getCols() + "]");

    Matrix sum = new Matrix(getRows(), getCols());
    double[] entries = new double[getRows() * getCols()];

    for (int row = 0; row < getRows(); row++)
      for (int col = 0; col < getCols(); col++)
        entries[row * getCols() + col] = matrix[row][col] + other.get(row, col);

    sum.fill(entries);
    return sum;
  }



  /**
   * Subtract, from this matrix, another. Operands remain unchanged.
   *
   * @param other Second operand
   * @return Difference matrix
   * @throws RuntimeException If the two matrices are not eligible for subtraction
   */
  public Matrix subtract(Matrix other) {
    if (checkArithmetic && getCols() != other.getCols() || getRows() != other.getRows())
      throw new RuntimeException("Illegal subtraction on matrices of sizes [" + getRows() + ", " + getCols() + "] and [" + other.getRows() + ", " + other.getCols() + "]");

    Matrix difference = new Matrix(getRows(), getCols());
    double[] entries = new double[getRows() * getCols()];

    for (int row = 0; row < getRows(); row++)
      for (int col = 0; col < getCols(); col++)
        entries[row * getCols() + col] = matrix[row][col] - other.get(row, col);

    difference.fill(entries);
    return difference;
  }



  /**
   * Transpose this matrix. Original remains unchanged.
   *
   * @return Transposed matrix
   */
  public Matrix transpose() {
    Matrix mat = new Matrix(getCols(), getRows());
    double[] entries = new double[getRows() * getCols()];

    for (int row = 0; row < getRows(); row++)
      for (int col = 0; col < getCols(); col++)
        entries[col * getRows() + row] = matrix[row][col];

    mat.fill(entries);
    return mat;
  }



  /**
   * Generate a covariance matrix from a matrix consisting of a single column
   *
   * @return Covariance matrix
   */
  public Matrix getCovarianceMatrix() {
    if (getCols() != 1)
      throw new RuntimeException("Cannot generate a covariance matrix from a matrix with >1 column");

    Matrix mat = new Matrix(getRows(), getRows());
    double[] entries = new double[getRows() * getRows()];

    for (int row = 0; row < getRows(); row++)
      for (int col = 0; col < getRows(); col++)
        entries[col * getRows() + row] = matrix[row][0] * matrix[col][0];

    mat.fill(entries);
    return mat;
  }



  /**
   * Clone this matrix
   *
   * @return Identical matrix with unique memory address
   */
  @Override public Matrix clone() {
    Matrix clone = new Matrix(getRows(), getCols());
    double entries[] = new double[getRows() * getCols()];

    for (int row = 0; row < getRows(); row++)
      for (int col = 0; col < getCols(); col++)
        entries[row * getCols() + col] = matrix[row][col];

    clone.fill(entries);
    return clone;
  }



  /**
   * Clone this matrix and specify the clone's arithmetic check
   *
   * @return Identical matrix with unique memory address and specified arithmetic check
   */
  public Matrix clone(boolean checkArithmetic) {
    Matrix clone = new Matrix(getRows(), getCols());
    double entries[] = new double[getRows() * getCols()];

    for (int row = 0; row < getRows(); row++)
      for (int col = 0; col < getCols(); col++)
        entries[row * getCols() + col] = matrix[row][col];

    clone.fill(entries);
    clone.checkArithmetic = checkArithmetic;
    return clone;
  }



  /**
   * Format this matrix into a grid. Column width can be specified via Matrix.setFormatColumnWidth()
   * and defaults to 10.
   *
   * @return String representation of this matrix
   * @see Matrix.setFormatColumnWidth()
   */
  @Override public String toString() {
    String str = "";

    for (double[] row : matrix) {
      for (double d : row)
        str += String.format("%" + formatColumnWidth + "s", d) + " ";
      str += "\n";
    }

    return str;
  }
}