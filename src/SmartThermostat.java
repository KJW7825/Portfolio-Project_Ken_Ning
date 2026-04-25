import components.pidcontroller.PIDController;
import components.pidcontroller.PIDController1;

/**
 * A thermostat using PIDController as a component.
 *
 * @author Ken Ning
 */
public class SmartThermostat {

    /**
     * Proportional tuning value.
     */
    private static final double KP_VALUE = 2.0;

    /**
     * Integral tuning value.
     */
    private static final double KI_VALUE = 0.5;

    /**
     * Derivative tuning value.
     */
    private static final double KD_VALUE = 1.0;

    /**
     * The PID controller component.
     */
    private PIDController pid;

    /**
     * The current temperature of the room.
     */
    private double currentTemp;

    /**
     * Sets up the thermostat.
     *
     * @param initialTemp
     *            starting room temperature
     */
    public SmartThermostat(double initialTemp) {
        this.pid = new PIDController1();
        this.pid.setGains(KP_VALUE, KI_VALUE, KD_VALUE);
        this.currentTemp = initialTemp;
    }

    /**
     * Adjusts temperature towards the target.
     *
     * @param targetTemp
     *            desired temperature
     */
    public void updateTemperature(double targetTemp) {
        double error = targetTemp - this.currentTemp;
        double power = this.pid.calculateOutput(error);
        this.currentTemp = this.currentTemp + power;
    }

    /**
     * Gets the current temperature.
     *
     * @return current temperature
     */
    public double getCurrentTemperature() {
        return this.currentTemp;
    }
}
