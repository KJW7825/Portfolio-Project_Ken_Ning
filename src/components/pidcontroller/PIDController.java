package components.pidcontroller;

/**
 * Enhanced interface for a PID Controller. It inherits all standard and kernel
 * methods, and adds useful secondary methods.
 *
 * @author Ken Ning
 */
public interface PIDController extends PIDControllerKernel {

    /**
     * Clears out the internal memory (accumulated past mistakes and the
     * previous mistake), but keeps the tuning values (Kp, Ki, Kd) exactly as
     * they are.
     */
    void resetMemory();

}
