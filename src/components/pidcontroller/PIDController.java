package components.pidcontroller;

/**
 * The Enhanced Interface for the PID Controller.
 *
 * @author Ken Ning
 */
public interface PIDController extends PIDControllerKernel {

    /**
     * Wipes the controller's error memory (integral and last error) back to
     * zero. The tuning gains (Kp, Ki, Kd) remain unchanged.
     *
     * @updates this
     */
    void resetMemory();

}
