package components.pidcontroller;

/**
 * Layered implementations of secondary methods for {@code PIDController}.
 *
 * @author Ken Ning
 */
public abstract class PIDControllerSecondary implements PIDController {

    /**
     * The prime number used to generate unique hash codes. Defined as a
     * constant to avoid magic number Checkstyle errors.
     */
    private static final int HASH_MULTIPLIER = 31;

    /*
     * Secondary methods ------------------------------------------------------
     */

    @Override
    public final void resetMemory() {
        // Step 1: Save the current Kp, Ki, and Kd values so we do not lose them
        double savedP = this.getKp();
        double savedI = this.getKi();
        double savedD = this.getKd();

        // Step 2: Use the manual clear() method from the Kernel to wipe everything
        this.clear();

        // Step 3: Put the saved Kp, Ki, and Kd values back into the controller
        this.setGains(savedP, savedI, savedD);
    }

    /*
     * Common Object methods --------------------------------------------------
     */

    @Override
    public final boolean equals(Object obj) {
        // Check if looking at the exact same object
        if (obj == this) {
            return true;
        }

        // Check if the other object is empty (null)
        if (obj == null) {
            return false;
        }

        // Check if the other object is the same type of component
        if (!(obj instanceof PIDController)) {
            return false;
        }

        // Cast the object so we can read its values
        PIDController other = (PIDController) obj;

        // Compare the core parameters using Kernel getters
        boolean sameKp = (this.getKp() == other.getKp());
        boolean sameKi = (this.getKi() == other.getKi());
        boolean sameKd = (this.getKd() == other.getKd());

        // They are equal ONLY if all gains match
        return sameKp && sameKi && sameKd;
    }

    @Override
    public final int hashCode() {
        // Start with the base number (using the constant defined at the top)
        int result = HASH_MULTIPLIER;

        // Mix in the hash values of our three gains
        result = HASH_MULTIPLIER * result + Double.hashCode(this.getKp());
        result = HASH_MULTIPLIER * result + Double.hashCode(this.getKi());
        result = HASH_MULTIPLIER * result + Double.hashCode(this.getKd());

        return result;
    }

    @Override
    public final String toString() {
        // Use StringBuilder to stick text pieces together efficiently
        StringBuilder text = new StringBuilder();

        text.append("<PIDController: Kp=");
        text.append(this.getKp());
        text.append(", Ki=");
        text.append(this.getKi());
        text.append(", Kd=");
        text.append(this.getKd());
        text.append(">");

        return text.toString();
    }
}
