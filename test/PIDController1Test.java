import static org.junit.Assert.assertEquals;

import org.junit.Test;

import components.pidcontroller.PIDController;
import components.pidcontroller.PIDController1;

/**
 * JUnit test fixture for {@code PIDController1}.
 *
 * @author Ken Ning
 */
public class PIDController1Test {

    // Constant for double comparison tolerance
    private static final double DELTA = 0.0001;

    /**
     * Helper method to create a new, empty controller.
     *
     * @return a new PIDController instance
     */
    private PIDController constructorTest() {
        return new PIDController1();
    }

    /**
     * Helper method to create a controller with specific gains set.
     *
     * @param p
     *            Proportional gain
     * @param i
     *            Integral gain
     * @param d
     *            Derivative gain
     * @return a configured PIDController instance
     */
    private PIDController constructorRef(double p, double i, double d) {
        PIDController controller = new PIDController1();
        controller.setGains(p, i, d);
        return controller;
    }

    /*
     * Tests for Constructor and Standard Methods ----------------------------
     */

    @Test
    public final void testConstructor() {
        PIDController controller = this.constructorTest();

        // A brand new controller should have all gains set to 0.0
        assertEquals(0.0, controller.getKp(), DELTA);
        assertEquals(0.0, controller.getKi(), DELTA);
        assertEquals(0.0, controller.getKd(), DELTA);
    }

    @Test
    public final void testNewInstance() {
        PIDController controller = this.constructorRef(1.0, 0.5, 0.1);
        PIDController copy = controller.newInstance();

        // The new instance should be a completely clean slate
        assertEquals(0.0, copy.getKp(), DELTA);
        assertEquals(0.0, copy.getKi(), DELTA);
        assertEquals(0.0, copy.getKd(), DELTA);

        // Ensure original controller is untouched
        assertEquals(1.0, controller.getKp(), DELTA);
    }

    @Test
    public final void testClear() {
        PIDController controller = this.constructorRef(2.0, 1.0, 0.5);

        // Wipe the memory
        controller.clear();

        // Everything should be back to zero
        assertEquals(0.0, controller.getKp(), DELTA);
        assertEquals(0.0, controller.getKi(), DELTA);
        assertEquals(0.0, controller.getKd(), DELTA);
    }

    @Test
    public final void testTransferFrom() {
        PIDController controller1 = this.constructorRef(3.0, 2.0, 1.0);
        PIDController controller2 = this.constructorTest();

        // controller2 steals data from controller1
        controller2.transferFrom(controller1);

        // controller2 should now have the data
        assertEquals(3.0, controller2.getKp(), DELTA);
        assertEquals(2.0, controller2.getKi(), DELTA);
        assertEquals(1.0, controller2.getKd(), DELTA);

        // controller1 should be wiped clean
        assertEquals(0.0, controller1.getKp(), DELTA);
        assertEquals(0.0, controller1.getKi(), DELTA);
        assertEquals(0.0, controller1.getKd(), DELTA);
    }

    /*
     * Tests for Kernel Methods ----------------------------------------------
     */

    @Test
    public final void testSetGains() {
        PIDController controller = this.constructorTest();

        controller.setGains(1.5, 0.75, 0.25);

        assertEquals(1.5, controller.getKp(), DELTA);
        assertEquals(0.75, controller.getKi(), DELTA);
        assertEquals(0.25, controller.getKd(), DELTA);
    }

    @Test
    public final void testCalculateOutputProportionalOnly() {
        PIDController controller = this.constructorTest();
        // Only turn on the P (Proportional) brain
        controller.setGains(2.0, 0.0, 0.0);

        // Mistake is 10. Output should be 10 * 2.0 = 20.0
        double output = controller.calculateOutput(10.0);

        assertEquals(20.0, output, DELTA);
    }

    @Test
    public final void testCalculateOutputProportionalAndIntegral() {
        PIDController controller = this.constructorTest();
        // Turn on P and I
        controller.setGains(1.0, 0.5, 0.0);

        // Step 1: Mistake is 10.
        // P = 10 * 1.0 = 10.
        // I (Memory) = 10. I_calc = 10 * 0.5 = 5.
        // Total = 15.0
        double output1 = controller.calculateOutput(10.0);
        assertEquals(15.0, output1, DELTA);

        // Step 2: Mistake is still 10.
        // P = 10 * 1.0 = 10.
        // I (Memory) = 10 + 10 = 20. I_calc = 20 * 0.5 = 10.
        // Total = 20.0
        double output2 = controller.calculateOutput(10.0);
        assertEquals(20.0, output2, DELTA);
    }

    /*
     * Tests for Secondary Methods -------------------------------------------
     */

    @Test
    public final void testResetMemory() {
        PIDController controller = this.constructorRef(1.0, 2.0, 3.0);

        // Push some data through so the controller remembers past mistakes
        controller.calculateOutput(10.0);
        controller.calculateOutput(10.0);

        // Reset the memory of past mistakes
        controller.resetMemory();

        // The tuning values (gains) should remain untouched
        assertEquals(1.0, controller.getKp(), DELTA);
        assertEquals(2.0, controller.getKi(), DELTA);
        assertEquals(3.0, controller.getKd(), DELTA);

        // Next calculation should act like it's the very first time
        // Mistake 10, Kp=1, Ki=2, Kd=3.
        // P = 10 * 1 = 10
        // I = 10 * 2 = 20
        // D = (10 - 0) * 3 = 30
        // Total = 60
        double output = controller.calculateOutput(10.0);
        assertEquals(60.0, output, DELTA);
    }
}
