package components.pidcontroller;

import components.standard.Standard;

/**
 * The Kernel Interface for the PID Controller. * Think of this as the "Brain"
 * of a car's cruise control. It keeps the car at a steady speed by looking at
 * the current gap (error). * @author Ken Ning * @mathmodel type
 * PIDControllerKernel is modeled by ( kp: real, ki: real, kd: real,
 * integralSum: real, lastError: real )
 *
 * @initially {@code
 * ():
 * ensures this = (0.0, 0.0, 0.0, 0.0, 0.0)
 * }
 */
public interface PIDControllerKernel extends Standard<PIDController> {

    /**
     * Sets the "personality" of the controller using three tuning knobs.
     * * @param p Proportional gain: How hard to push based on the CURRENT gap.
     *
     * @param i
     *            Integral gain: How much to push based on PAST mistakes.
     * @param d
     *            Derivative gain: How much to brake based on FUTURE
     *            predictions.
     * @updates this
     * @requires p >= 0 and i >= 0 and d >= 0
     * @ensures this.kp = p and this.ki = i and this.kd = d
     */
    void setGains(double p, double i, double d);

    /**
     * Calculates the action to take (like pressing the gas pedal) based on the
     * current gap. * @param currentError The difference between the target
     * speed and the actual speed.
     *
     * @return The calculated output/action to take.
     * @updates this
     * @ensures calculateOutput = (this.kp * currentError) + (this.ki *
     *          (this.integralSum + currentError)) + (this.kd * (currentError -
     *          this.lastError))
     */
    double calculateOutput(double currentError);

}
