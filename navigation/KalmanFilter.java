package org.firstinspires.ftc.teamcode.vv7797.opmode.navigation;

/**
 * Modular, physics-based Kalman filter. Supports filtering for one, two, and
 * three-dimensional states. Allows for black-and-white variant and covariant
 * error processing, though partial covariance could still be achieved with
 * clever manipulation of matrix arithmetic.
 */

import org.firstinspires.ftc.teamcode.vv7797.opmode.util.Matrix;
import org.firstinspires.ftc.teamcode.vv7797.opmode.util.MatrixBuilder;

public class KalmanFilter {

  public enum StateDimensions {
    S1D(1),
    S2D(2),
    S3D(3);

    private int ordinal;

    StateDimensions(int ordinal) {
      this.ordinal = ordinal;
    }

    public int asInt() {
      return ordinal;
    }
  }



  private final Matrix I_1D = new Matrix(2, 2,
          1, 0,
          0, 1
  );
  private final Matrix I_2D = new Matrix(4, 4,
          1, 0, 0, 0,
          0, 1, 0, 0,
          0, 0, 1, 0,
          0, 0, 0, 1
  );
  private final Matrix I_3D = new Matrix(6, 6,
          1, 0, 0, 0, 0, 0,
          0, 1, 0, 0, 0, 0,
          0, 0, 1, 0, 0, 0,
          0, 0, 0, 1, 0, 0,
          0, 0, 0, 0, 1, 0,
          0, 0, 0, 0, 0, 1
  );



  private final Matrix Z_1D = new Matrix(2, 1,
          0,
          0
  );
  private final Matrix Z_2D = new Matrix(4, 1,
          0,
          0,
          0,
          0
  );
  private final Matrix Z_3D = new Matrix(6, 1,
          0,
          0,
          0,
          0,
          0,
          0
  );



  private StateDimensions dimensions;
  private MatrixBuilder matrixBuilder;
  private Matrix A, B, I, Z;
  private Matrix stateUpdates, obsErr, procErr;
  private Matrix stateCurrent, statePrevious;
  private double T_INTERV;
  private boolean SIMP_COV_MATS;



  /**
   * @param timeInterval Approximate time between filter iterations (unit-agnostic, though assumed constant)
   * @param simpCovMats Whether or not to reduce covariance-variance matrices into just variance matrices
   * @param dims The dimension of the states being filtered
   */
  public KalmanFilter(double timeInterval, boolean simpCovMats, StateDimensions dims, MatrixBuilder builder) {
    T_INTERV = timeInterval;
    SIMP_COV_MATS = simpCovMats;
    dimensions = dims;
    matrixBuilder = builder;

    setTimeInterval(timeInterval);
  }



  /**
   * Set the TIME interval and build necessary matrices
   *
   * @param interval Approximate TIME between filter iterations (unit-agnostic, though assumed constant)
   */
  public void setTimeInterval(double interval) {
    T_INTERV = interval;

    switch (dimensions) {
      case S1D:
        A = matrixBuilder.build (2, 2,
                1, T_INTERV,
                0,        1
        );
        B = matrixBuilder.build(2, 2,
                0.5 * Math.pow(T_INTERV, 2),
                T_INTERV
        );
        I = I_1D.clone(matrixBuilder.getCheckArithmetic());
        Z = Z_1D.clone(matrixBuilder.getCheckArithmetic());
        break;

      case S2D:
        A = matrixBuilder.build(4, 4,
                1,        0, T_INTERV,        0,
                0,        1,        0, T_INTERV,
                0,        0,        1,        0,
                0,        0,        0,        1
        );
        B = matrixBuilder.build(4, 1,
                0.5 * Math.pow(T_INTERV, 2),
                0.5 * Math.pow(T_INTERV, 2),
                T_INTERV,
                T_INTERV
        );
        I = I_2D.clone(matrixBuilder.getCheckArithmetic());
        Z = Z_2D.clone(matrixBuilder.getCheckArithmetic());
        break;

      case S3D:
        A = matrixBuilder.build(6, 6,
                1,        0,        0, T_INTERV,        0,        0,
                0,        1,        0,        0, T_INTERV,        0,
                0,        0,        1,        0,        0, T_INTERV,
                0,        0,        0,        1,        0,        0,
                0,        0,        0,        0,        1,        0,
                0,        0,        0,        0,        0,        1
        );
        B = matrixBuilder.build(6, 1,
                0.5 * Math.pow(T_INTERV, 2),
                0.5 * Math.pow(T_INTERV, 2),
                0.5 * Math.pow(T_INTERV, 2),
                T_INTERV,
                T_INTERV,
                T_INTERV
        );
        I = I_3D.clone(matrixBuilder.getCheckArithmetic());
        Z = Z_3D.clone(matrixBuilder.getCheckArithmetic());
        break;
    }
  }



  public void setCurrentState(Matrix state) {
    stateCurrent = state.clone();
  }

  public void setPreviousState(Matrix state) {
    statePrevious = state.clone();
  }

  public void setObservationErrors(double xPosObsErr, double xVelObsErr) {
    obsErr = matrixBuilder.build(2, 1, xPosObsErr, xVelObsErr);
  }

  public void setObservationErrors(double xPosObsErr, double yPosObsErr, double xVelObsErr, double yVelObsErr) {
    obsErr = matrixBuilder.build(4, 1, xPosObsErr, yPosObsErr, xVelObsErr, yVelObsErr);
  }

  public void setObservationErrors(double xPosObsErr, double yPosObsErr, double zPosObsErr, double xVelObsErr, double yVelObsErr, double zVelObsErr) {
    obsErr = matrixBuilder.build(6, 1, xPosObsErr, yPosObsErr, zPosObsErr, xVelObsErr, yVelObsErr, zVelObsErr);
  }

  public void setProcessErrors(double xPosProcErr, double xVelProcErr) {
    procErr = matrixBuilder.build(2, 1, xPosProcErr, xVelProcErr);
  }

  public void setProcessErrors(double xPosProcErr, double yPosProcErr, double xVelProcErr, double yVelProcErr) {
    procErr = matrixBuilder.build(4, 1, xPosProcErr, yPosProcErr, xVelProcErr, yVelProcErr);
  }

  public void setProcessErrors(double xPosProcErr, double yPosProcErr, double zPosProcErr, double xVelProcErr, double yVelProcErr, double zVelProcErr) {
    procErr = matrixBuilder.build(6, 1, xPosProcErr, yPosProcErr, zPosProcErr, xVelProcErr, yVelProcErr, zVelProcErr);
  }

  public void setStateUpdates(double xStateUpd) {
    stateUpdates = matrixBuilder.build(1, 1, xStateUpd);
  }

  public void setStateUpdates(double xStateUpd, double yStateUpd) {
    stateUpdates = matrixBuilder.build(2, 1, xStateUpd, yStateUpd);
  }

  public void setStateUpdates(double xStateUpd, double yStateUpd, double zStateUpd) {
    stateUpdates = matrixBuilder.build(3, 1, xStateUpd, yStateUpd, zStateUpd);
  }



  /**
   * @return The number of dimensions this filter is filtering
   */
  public StateDimensions getStateDimensions() {
    return dimensions;
  }



  /**
   * @return Filtered state
   */
  public Matrix filter() {
    // Establish constants and predict state
    Matrix stateUpdate;
    double xStateUpd, yStateUpd, zStateUpd;

    switch (dimensions) {
      case S1D:
        xStateUpd = stateUpdates.get(0, 0);
        stateUpdate = matrixBuilder.build(2, 1, xStateUpd, xStateUpd);
        break;

      case S2D:
        xStateUpd = stateUpdates.get(0, 0);
        yStateUpd = stateUpdates.get(1, 0);
        stateUpdate = matrixBuilder.build(4, 1, xStateUpd, yStateUpd, xStateUpd, yStateUpd);
        break;

      case S3D:
        xStateUpd = stateUpdates.get(0, 0);
        yStateUpd = stateUpdates.get(1, 0);
        zStateUpd = stateUpdates.get(2, 0);
        stateUpdate = matrixBuilder.build(6, 1, xStateUpd, yStateUpd, zStateUpd, xStateUpd, yStateUpd, zStateUpd);
        break;

      default:
        stateUpdate = null;
        break;
    }

    Matrix statePred = A.multiply(statePrevious).add(B.multiply(stateUpdate));

    // Process covariance
    Matrix procCov = procErr.getCovarianceMatrix();
    procCov = simplifyCovMat(procCov);

    // Observation covariance
    Matrix obsCov = obsErr.getCovarianceMatrix();
    obsCov = simplifyCovMat(obsCov);

    // Predicted process covariance
    Matrix predProcCov = A.multiply(procCov).multiply(A.transpose());
    predProcCov = simplifyCovMat(predProcCov);

    // Kalman Gain
    Matrix kg = predProcCov.multiply(I.transpose()).divide(I.multiply(predProcCov).multiply(I.transpose()).add(obsCov));
    kg = simplifyCovMat(kg);

    // New observation
    Matrix obsNew = I.multiply(stateCurrent).add(Z);

    // Update current state
    stateCurrent = statePred.add(kg.multiply(obsNew.subtract(statePred)));

    // Update process covariance
    procCov = I.subtract(kg.multiply(I)).multiply(predProcCov);

    // Update previous state
    statePrevious = stateCurrent.clone();

    // Return final calculation
    return stateCurrent;
  }



  /**
   * If SIMP_COV_MATS was made true for this filter, all covariances become 0
   *
   * @param mat Variance-covariance matrix
   * @return Variance matrix
   */
  public Matrix simplifyCovMat(Matrix mat) {
    if (!SIMP_COV_MATS)
      return mat;

    Matrix simple = matrixBuilder.build(mat.getRows(), mat.getCols());
    double[] entries = new double[mat.getRows() * mat.getCols()];

    for (int row = 0; row < mat.getRows(); row++)
      for (int col = 0; col < mat.getCols(); col++)
        if (row == col)
          entries[row * mat.getRows() + col] = (Double.isNaN(mat.get(row, col)) ? 0 : mat.get(row, col));

    simple.fill(entries);
    return simple;
  }
}