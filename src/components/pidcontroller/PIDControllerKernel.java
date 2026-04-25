package components.pidcontroller;

import components.standard.Standard;

/**
 * Core methods for a PID Controller. This kernel interface defines the basic
 * mathematical model for the controller.
 *
 * @author Ken Ning
 */
public interface PIDControllerKernel extends Standard<PIDController> {

    /**
     * Sets all three tuning values (gains) at the same time.
     *
     * @param p
     *            the new Proportional value (Kp)
     * @param i
     *            the new Integral value (Ki)
     * @param d
     *            the new Derivative value (Kd)
     */
    void setGains(double p, double i, double d);

    /**
     * Calculates the machine adjustment needed based on the current mistake.
     *
     * @param error
     *            the difference between our target and our current state
     * @return the calculated output adjustment
     */
    double calculateOutput(double currentError);

    /**
     * Gets the current Proportional gain (Kp).
     *
     * @return the Kp value
     */
    double getKp();

    /**
     * Gets the current Integral gain (Ki).
     *
     * @return the Ki value
     */
    double getKi();

    /**
     * Gets the current Derivative gain (Kd).
     *
     * @return the Kd value
     */
    double getKd();
}
