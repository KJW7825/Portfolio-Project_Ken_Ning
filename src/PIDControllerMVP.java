/**
 * A Proof-of-Concept Minimum Viable Product (MVP) for the PIDController
 * component. * What is a PID Controller? Imagine driving a car and trying to
 * keep the speed exactly at 60 mph. A PID controller is an algorithm that
 * calculates how hard to press the gas pedal to reach and maintain that exact
 * speed without going over. * The underlying mathematical formula is: Output =
 * Kp * e(t) + Ki * ∫ e(t)dt + Kd * de(t)/dt * In simple terms: Output = (P *
 * current_error) + (I * sum_of_past_errors) + (D * change_in_error)
 *
 * @author Ken Ning
 */
public final class PIDControllerMVP {

    /*
     * Private Members (Internal Memory) These variables store the "personality"
     * and the "memory" of our controller.
     */

    /**
     * The Proportional gain (P). Reacts to the present moment to decide how
     * aggressively to push.
     */
    private double kp;

    /**
     * The Integral gain (I). Remembers the past by accumulating errors to fix
     * lingering tiny gaps.
     */
    private double ki;

    /**
     * The Derivative gain (D). Predicts the future by looking at the rate of
     * change to avoid overshooting.
     */
    private double kd;

    /**
     * The controller's memory notebook for past mistakes. A running total of
     * all previous errors, used by the Integral term.
     */
    private double accumulatedIntegral;

    /**
     * The mistake we made just a moment ago. Used by the Derivative term to
     * calculate how fast the error is changing.
     */
    private double previousError;

    /**
     * No-argument constructor. Initializes a brand new, blank-slate PID
     * controller.
     */
    public PIDControllerMVP() {
        /*
         * Reset all tuning knobs and memory back to zero.
         */
        this.kp = 0.0;
        this.ki = 0.0;
        this.kd = 0.0;
        this.accumulatedIntegral = 0.0;
        this.previousError = 0.0;
    }

    /*
     * ---------------------------------------------------------------- Kernel
     * Methods (Basic operations to read/write data)
     * ----------------------------------------------------------------
     */

    /**
     * Sets the tuning knobs for how the controller should behave.
     *
     * @param newKp
     *            the Proportional gain (Present focus)
     * @param newKi
     *            the Integral gain (Past focus)
     * @param newKd
     *            the Derivative gain (Future focus)
     */
    public void setGains(double newKp, double newKi, double newKd) {
        /*
         * Save the provided settings into our internal variables.
         */
        this.kp = newKp;
        this.ki = newKi;
        this.kd = newKd;
    }

    /**
     * Updates the controller's memory with what just happened.
     *
     * @param currentError
     *            how far off we currently are from our goal
     */
    public void recordError(double currentError) {
        /*
         * Add the current mistake to our running total. If we stay slightly
         * below our target speed for a long time, this total will grow huge and
         * force the car to speed up.
         */
        this.accumulatedIntegral = this.accumulatedIntegral + currentError;

        /*
         * Write down the current mistake so we can compare it next time. This
         * helps us know if things are getting better or worse.
         */
        this.previousError = currentError;
    }

    /*
     * ----------------------------------------------------------------
     * Secondary Methods (Complex logic built on top of kernel methods)
     * ----------------------------------------------------------------
     */

    /**
     * Erases the controller's memory. Useful if you want to completely restart
     * the task.
     */
    public void resetState() {
        /*
         * Wipe the memory clean.
         */
        this.accumulatedIntegral = 0.0;
        this.previousError = 0.0;
    }

    /**
     * The brain of the controller. It looks at how far off we are from the goal
     * and decides what action to take right now.
     *
     * @param currentError
     *            the gap between where we are and where we want to be
     * @return the calculated action to take (e.g., how hard to press the gas)
     */
    public double calculateOutput(double currentError) {

        /*
         * Step 1: The "P" (Proportional) Action -> "Look at the PRESENT"
         * Analogy: If we are 20 mph below the speed limit, press the gas pedal
         * hard. If we are only 2 mph below, press it gently.
         */
        double pTerm = this.kp * currentError;

        /*
         * Step 2: The "I" (Integral) Action -> "Look at the PAST" Analogy: Are
         * we stuck at 59 mph for a whole minute? The P-action is too weak to
         * push us that last 1 mph. The I-action remembers this frustration,
         * builds up over time, and says "Just give it a little more gas!"
         */
        double iTerm = this.ki * (this.accumulatedIntegral + currentError);

        /*
         * Step 3: The "D" (Derivative) Action -> "Look at the FUTURE" Analogy:
         * We were at 40 mph, now we are at 55 mph. We are speeding up really
         * fast! The D-action notices this rapid change and says "Whoa, ease off
         * the gas, otherwise we will blast past 60 mph!"
         */
        double dTerm = this.kd * (currentError - this.previousError);

        /*
         * Step 4: Combine all three advice columns to make the final decision.
         */
        double totalOutput = pTerm + iTerm + dTerm;

        /*
         * Step 5: Before finishing, update our memory notebook with the current
         * situation so we are ready for the next time we need to decide.
         */
        this.recordError(currentError);

        return totalOutput;
    }

    /*
     * ---------------------------------------------------------------- Main
     * Method (Testing the component to prove it works)
     * ----------------------------------------------------------------
     */

    /**
     * Main method to demonstrate the component in action. * We will simulate a
     * car trying to use Cruise Control to stay exactly at 60 mph.
     *
     * @param args
     *            command-line arguments (not used)
     */
    public static void main(String[] args) {

        // Step 1: Buy a new PID controller for our car.
        PIDControllerMVP cruiseControl = new PIDControllerMVP();

        // Step 2: Tune the controller.
        // P=0.25 (faster initial acceleration)
        // I=0.001 (very small, to prevent "Integral Windup" overshooting)
        // D=0.15 (stronger brake to smoothly ease into the target speed)
        // This is manual setting value will cause our final result different
        // This is the key of the whole code
        // First When I try "cruiseControl.setGains(0.1, 0.01, 0.05);"
        // It will make the speed to almost 76mph
        // Then Fix it with value change
        cruiseControl.setGains(0.25, 0.001, 0.15);

        // Step 3: Set our desired target speed.
        double targetSpeed = 60.0;

        // Step 4: Start the car at a complete stop.
        double currentSpeed = 0.0;

        System.out.println("Starting Cruise Control Simulation...");
        System.out.println("Target Speed: " + targetSpeed + " mph\n");

        // Simulate 20 moments in time (ticks).
        for (int timeStep = 1; timeStep <= 20; timeStep++) {

            // Calculate our mistake: How far are we from 60 mph?
            double error = targetSpeed - currentSpeed;

            // Ask the PID controller: "How much gas should I give it?"
            double throttle = cruiseControl.calculateOutput(error);

            // Apply the gas to the car.
            // (Assume 1 unit of throttle increases speed by 1 mph).
            currentSpeed = currentSpeed + throttle;

            // Print the dashboard readings so we can watch the speed smooth out.
            // Print the dashboard readings so we can watch the speed smooth out.
            System.out.printf(
                    "Time Step %2d: Mistake Gap = %5.2f | "
                            + "Gas Pedal Push = %5.2f | "
                            + "Current Speed = %5.2f mph%n",
                    timeStep, error, throttle, currentSpeed);
        }

        System.out.println("\nSimulation complete. "
                + "The car smoothly reached and maintained the target speed!");
    }
}
