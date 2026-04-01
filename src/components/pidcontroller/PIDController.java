package components.pidcontroller;

/**
 * The Enhanced Interface for the PID Controller.
 */
public interface PIDController extends PIDControllerKernel {

    /**
     * Wipes the controller's error memory (integral and last error). * @updates
     * this
     *
     * @ensures this.integralSum = 0.0 and this.lastError = 0.0
     */
    void resetMemory();

    /**
     * Reports the current Proportional (P) gain setting.
     *
     * @return the current value of the kp parameter
     * @ensures getKp = this.kp
     */
    double getKp();

    /**
     * Reports the current Integral (I) gain setting.
     *
     * @return the current value of the ki parameter
     * @ensures getKi = this.ki
     */
    double getKi();

    /**
     * Reports the current Derivative (D) setting.
     *
     * @return the current value of the kd parameter
     * @ensures getKd = this.kd
     */
    double getKd();
}
