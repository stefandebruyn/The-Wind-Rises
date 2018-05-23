package org.firstinspires.ftc.teamcode.vv7797.opmode.control;

/**
 * Uses two separate PID controllers to correct error, delegating corrections to one of the two depending on where the error
 * is relative to some specified threshold
 */
public class PIDManager implements PIDFramework, Cloneable {
    public enum IntegralCalculation { AVERAGE, ALPHA, BETA }
    private IntegralCalculation integralCalculation;
    private final PIDController alpha, beta;
    private final double toggleThreshold;



    /**
     * @param alpha Controller for correcting error within the threshold
     * @param beta Controller fo correcting error outside of the threshold
     * @param toggleThreshold Error threshold
     * @param integralCalculation Method for calculating integral components
     */
    public PIDManager(PIDController alpha, PIDController beta, double toggleThreshold, IntegralCalculation integralCalculation) {
        this.alpha = alpha;
        this.beta = beta;
        this.toggleThreshold = toggleThreshold;
        this.integralCalculation = integralCalculation;
    }



    /**
     * Clear error memory
     */
    public void reset() {
        alpha.reset();
        beta.reset();
    }



    /**
     * @param error System error
     * @return Proportional component
     */
    public double getP(double error) { return (Math.abs(error) < toggleThreshold ? alpha.getP(error) : beta.getP(error)); }



    /**
     * @return Integral component
     */
    public double getI() {
        switch (integralCalculation) {
            case AVERAGE:
                return (alpha.getI() + beta.getI()) / 2;

            case ALPHA:
                return alpha.getI();

            case BETA:
                return beta.getI();

            default:
                return 0;
        }
    }



    /**
     * @param error System error
     * @return Derivative component
     */
    public double getD(double error) { return (Math.abs(error) < toggleThreshold ? alpha.getP(error) : beta.getP(error)); }



    /**
     * Get manager output
     *
     * @param error System error
     * @return Output
     */
    public double getCorrection(double error) { return getP(error) + getI() + getD(error); }



    /**
     * @return Identical manager
     */
    @Override public PIDManager clone() { return new PIDManager(alpha.clone(), beta.clone(), toggleThreshold, integralCalculation); }
}
