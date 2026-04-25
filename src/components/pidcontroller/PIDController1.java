package components.pidcontroller;

/**
 * Kernel implementation of {@code PIDController} using basic Java doubles.
 *
 * @convention <pre>
 * $this.kp >= 0.0 and
 * $this.ki >= 0.0 and
 * $this.kd >= 0.0
 * </pre>
 * @correspondence <pre>
 * this = (kp: $this.kp, ki: $this.ki, kd: $this.kd,
 * accumulatedError: $this.accumulatedError,
 * previousError: $this.previousError)
 * </pre>
 *
 * @author Ken Ning
 */
public class PIDController1 extends PIDControllerSecondary {

    /*
     * Private members --------------------------------------------------------
     */

    /**
     * The Proportional gain (P). How hard we push based on the mistake right
     * now.
     */
    private double kp;

    /**
     * The Integral gain (I). Our memory of all past mistakes added up together.
     */
    private double ki;

    /**
     * The Derivative gain (D). Our prediction of the future to stop us from
     * going too fast.
     */
    private double kd;

    /**
     * The memory box that keeps adding up every mistake we make.
     */
    private double accumulatedError;

    /**
     * The last mistake we made just a second ago.
     */
    private double previousError;

    /**
     * Creator of initial representation. Clears all memory and sets every
     * number back to zero.
     */
    private void createNewRep() {
        this.kp = 0.0;
        this.ki = 0.0;
        this.kd = 0.0;
        this.accumulatedError = 0.0;
        this.previousError = 0.0;
    }

    /*
     * Constructors -----------------------------------------------------------
     */

    /**
     * No-argument constructor. Makes a brand new controller with nothing in its
     * memory.
     */
    public PIDController1() {
        // Start fresh with zero for everything
        this.createNewRep();
    }

    /*
     * Standard methods -------------------------------------------------------
     */

    @Override
    public final PIDController newInstance() {
        try {
            // Build a completely new, empty copy of this exact same tool
            return this.getClass().getConstructor().newInstance();
        } catch (ReflectiveOperationException e) {
            throw new AssertionError(
                    "Cannot construct object of type " + this.getClass());
        }
    }

    @Override
    public final void clear() {
        // Throw away all current memory and start completely fresh
        this.createNewRep();
    }

    @Override
    public final void transferFrom(PIDController source) {
        // Check to make sure the source is a real thing, and not ourselves
        assert source != null : "Violation of: source is not null";
        assert source != this : "Violation of: source is not this";
        assert source instanceof PIDController1 : ""
                + "Violation of: source is of dynamic type PIDController1";

        // Let Java know the source is exactly the same type as this object
        PIDController1 localSource = (PIDController1) source;

        // Steal all the numbers and memory from the source object
        this.kp = localSource.kp;
        this.ki = localSource.ki;
        this.kd = localSource.kd;
        this.accumulatedError = localSource.accumulatedError;
        this.previousError = localSource.previousError;

        // Wipe the source object completely clean so it is empty now
        localSource.createNewRep();
    }

    /*
     * Kernel methods ---------------------------------------------------------
     */

    @Override
    public final void setGains(double p, double i, double d) {
        // Make sure none of the numbers are below zero
        assert p >= 0.0 : "Violation of: p >= 0.0";
        assert i >= 0.0 : "Violation of: i >= 0.0";
        assert d >= 0.0 : "Violation of: d >= 0.0";

        // Save these new rules into our tool
        this.kp = p;
        this.ki = i;
        this.kd = d;
    }

    @Override
    public final double calculateOutput(double currentError) {
        // Action P (Present): Look at the mistake right now and multiply it
        double pTerm = this.kp * currentError;

        // Action I (Past): Add this mistake to our total memory, then multiply it
        this.accumulatedError = this.accumulatedError + currentError;
        double iTerm = this.ki * this.accumulatedError;

        // Action D (Future): Check mistake changed since last time, then multiply it
        double dTerm = this.kd * (currentError - this.previousError);

        // Put all three answers together to decide what to do
        double totalOutput = pTerm + iTerm + dTerm;

        // Write down this current mistake so we can look at it next time
        this.previousError = currentError;

        return totalOutput;
    }

    @Override
    public final double getKp() {
        // Give back the P number
        return this.kp;
    }

    @Override
    public final double getKi() {
        // Give back the I number
        return this.ki;
    }

    @Override
    public final double getKd() {
        // Give back the D number
        return this.kd;
    }
}
