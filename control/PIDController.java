package org.firstinspires.ftc.teamcode.vv7797.opmode.control;

import com.qualcomm.robotcore.util.ElapsedTime;

public class PIDController implements PIDFramework, Cloneable {
  private ElapsedTime runtime;
  private double kp, ki, kd;
  private double totalError, lastError, lastTime = -1;



  /**
   * @param kp Proportional gain
   * @param ki Integral gain
   * @param kd Derivative gain
   *
   * @param runtime Timer for use in derivative calculations
   */
  public PIDController(double kp, double ki, double kd, ElapsedTime runtime) {
    setGains(kp, ki, kd);
    this.runtime = runtime;
  }



  /**
   * @param kp Proportional gain
   * @param ki Integral gain
   * @param kd Derivative gain
   */
  public void setGains(double kp, double ki, double kd) {
    this.kp = kp;
    this.ki = ki;
    this.kd = kd;
  }



  /**
   * Clear error memory
   */
  public void reset() {
    totalError = lastError = 0;
    lastTime = -1;
  }



  /**
   * @param error System error
   * @return Proportional component
   */
  public double getP(double error) { return kp * error; }



  /**
   * @return Integral component
   */
  public double getI() { return ki * totalError; }



  /**
   * @param error System error
   * @return Derivative component
   */
  public double getD(double error) {
    try {
      return (lastTime == -1 ? 0 : kd * (error - lastError) / (runtime.time() - lastTime));
    } catch (NullPointerException e) {
      return 0;
    }
  }



  /**
   * Get controller output
   *
   * @param error System error
   * @return Output
   */
  public double getCorrection(double error) {
    totalError += error;

    final double output = getP(error) + getI() + getD(error);

    lastError = error;
    lastTime = runtime.time();

    return output;
  }



  /**
   * @return Identical controller
   */
  @Override public PIDController clone() { return new PIDController(kd, ki, kp, runtime); }
}
