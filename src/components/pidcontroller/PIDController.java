package components.pidcontroller;

/**
 * The Enhanced Interface for the PID Controller. * This adds convenience
 * features, like checking the current settings or wiping the controller's
 * "memory" clean.
 */
public interface PIDController extends PIDControllerKernel {

    /**
     * Wipes the controller's memory (past errors) without changing the P, I, D
     * settings. * Use this if you want the controller to start a new task from
     * scratch. * @updates this
     * @ensures this.integralSum = 0.0 and this.lastError = 0.0
     */
    void resetMemory();

    /**
     * Reports the current Proportional (P) setting. * @return the value of the
     * P gain.
     *
     * @ensures getKp = this.kp
     */
    double getKp();

    /**
     * Reports the current Integral (I) setting. * @return the value of the I
     * gain.
     *
     * @ensures getKi = this.ki
     */
    double getKi();

    /**
     * Reports the current Derivative (D) setting. * @return the value of the D
     * gain.
     *
     * @ensures getKd = this.kd
     */
    double getKd();
}
