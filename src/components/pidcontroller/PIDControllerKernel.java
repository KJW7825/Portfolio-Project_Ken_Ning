package components.pidcontroller;

/**
 * The Kernel Interface for the PID Controller.
 *
 * @author Ken Ning
 */
public interface PIDControllerKernel {

    /*
     * We temporarily remove 'extends Standard<PIDController>' to fix the
     * hierarchy error. We define clear() manually instead.
     */

    /**
     * Resets the controller to its initial state (all gains and errors to 0).
     *
     * @updates this
     * @ensures this is completely reset
     */
    void clear();

    /**
     * Sets the three main tuning parameters (Gains).
     *
     * @param p
     *            The Proportional gain
     * @param i
     *            The Integral gain
     * @param d
     *            The Derivative gain
     * @updates this
     * @requires p >= 0.0 and i >= 0.0 and d >= 0.0
     */
    void setGains(double p, double i, double d);

    /**
     * Calculates the control output based on the current system error.
     *
     * @param currentError
     *            The gap between target and actual value
     * @return The calculated action value
     * @updates this
     */
    double calculateOutput(double currentError);

    /**
     * Reports the current Proportional (P) gain setting.
     *
     * @return the kp value
     */
    double getKp();

    /**
     * Reports the current Integral (I) gain setting.
     *
     * @return the ki value
     */
    double getKi();

    /**
     * Reports the current Derivative (D) gain setting.
     *
     * @return the kd value
     */
    double getKd();
}
