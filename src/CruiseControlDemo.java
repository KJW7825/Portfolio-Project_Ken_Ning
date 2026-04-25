import components.pidcontroller.PIDController;
import components.pidcontroller.PIDController1;

/**
 * A simple demonstration of using the PIDController for car cruise control.
 * This acts as one of the required use cases for the portfolio project.
 *
 * @author Ken Ning
 */
public final class CruiseControlDemo {

    /**
     * Proportional tuning value.
     */
    private static final double KP_VALUE = 0.5;

    /**
     * Integral tuning value.
     */
    private static final double KI_VALUE = 0.1;

    /**
     * Derivative tuning value.
     */
    private static final double KD_VALUE = 0.05;

    /**
     * Target speed for the cruise control (mph).
     */
    private static final double TARGET_SPEED = 65.0;

    /**
     * Initial speed of the car (mph).
     */
    private static final double INITIAL_SPEED = 50.0;

    /**
     * Number of simulation steps.
     */
    private static final int SIMULATION_STEPS = 10;

    /**
     * Private constructor so this utility class cannot be instantiated.
     */
    private CruiseControlDemo() {
    }

    /**
     * Main method to run the demonstration.
     *
     * @param args
     *            the command line arguments
     */
    public static void main(String[] args) {
        PIDController controller = new PIDController1();

        // Set tuning values for speed control
        controller.setGains(KP_VALUE, KI_VALUE, KD_VALUE);

        double currentSpeed = INITIAL_SPEED;

        System.out.println("Starting Cruise Control Simulation...");
        System.out.println("Target Speed: " + TARGET_SPEED + " mph\n");

        for (int i = 1; i <= SIMULATION_STEPS; i++) {
            double error = TARGET_SPEED - currentSpeed;
            double throttle = controller.calculateOutput(error);
            currentSpeed = currentSpeed + throttle;

            System.out.printf(
                    "Time Step %2d: Speed = %5.2f mph | Gas Pedal = %5.2f%n", i,
                    currentSpeed, throttle);
        }

        System.out.println("\nSimulation complete.");
    }
}
