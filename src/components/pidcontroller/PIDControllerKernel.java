package components.pidcontroller;

import components.standard.Standard;

/**
 * The Kernel Interface for the PID Controller. * @author Ken Ning
 */
public interface PIDControllerKernel extends Standard<PIDController> {

    /**
     * Sets the three main tuning parameters (Gains) for the controller.
     * 
     * @param p
     *            The Proportional gain - reacts to the CURRENT error
     * @param i
     *            The Integral gain - reacts to the sum of PAST errors
     * @param d
     *            The Derivative gain - predicts FUTURE error trends
     * @updates this
     * @requires p >= 0 and i >= 0 and d >= 0
     * @ensures this.kp = p and this.ki = i and this.kd = d
     */
    void setGains(double p, double i, double d);

    /**
     * Calculates the control output based on the current system error.
     * 
     * @param currentError
     *            The gap between the target value and the actual value
     * @return The calculated action value to be applied to the system
     * @updates this
     * @ensures calculateOutput = (this.kp * currentError) + (this.ki *
     *          (this.integralSum + currentError)) + (this.kd * (currentError -
     *          this.lastError))
     */
    double calculateOutput(double currentError);

}
