package org.quixilver8404.powerplay.control;

public class PIDController {

    /**
     * Represents a constants kP, kI, and kD for a PID controller
     */
    public class PIDConstants {
        public final double kP; // Proportional constant
        public final double kI; // Integral constant
        public final double kD; // Derivative constant

        public PIDConstants(double kP, double kI, double kD) {
            this.kP = kP;
            this.kI = kI;
            this.kD = kD;
        }

        /**
         * Returns whether the input has the same kP, kI, and kD constants as this instance
         */
        public boolean isEqual(PIDConstants pidConstants) {
            return kP == pidConstants.kP && kI == pidConstants.kI && kD == pidConstants.kD;
        }
    }

    private PIDConstants pidConstants;

    private final int timeoutMillis; // Elapsed time at which this controller will reset

    private double integral = 0; // Cumulative integral
    private double prevError = 0; // The previous error, used for calculating the rate of change in error
    private long prevTimeMillis = 0; // The previous time in which the controller was looped, zero if never

    /**
     * Creates a new PID controller with the specified constants and timeout.
     *
     * @param timeoutMillis the time elapsed in milliseconds until the controller resets itself.
     */
    public PIDController(PIDConstants pidConstants, int timeoutMillis) {
        this.pidConstants = pidConstants;
        if (timeoutMillis <= 0) {
            this.timeoutMillis = Integer.MAX_VALUE;
        } else {
            this.timeoutMillis = timeoutMillis;
        }
    }

    /**
     * Creates a new PID controller with the specified constants and timeout.
     *
     * @param timeoutMillis the time elapsed in milliseconds until the controller resets itself.
     */
    public PIDController(double kP, double kI, double kD, int timeoutMillis) {
        pidConstants = new PIDConstants(kP, kI, kD);
        if (timeoutMillis <= 0) {
            this.timeoutMillis = Integer.MAX_VALUE;
        } else {
            this.timeoutMillis = timeoutMillis;
        }
    }

    /**
     * Sets new values for the constants P, I, and D. Returns whether or not values actually changed.
     */
    public boolean tune(PIDConstants newPIDConstants) {
        if (!pidConstants.isEqual(newPIDConstants)) {
            reset();
            pidConstants = newPIDConstants;
            return true;
        }
        return false;
    }

    /**
     * Sets new values for the constants P, I, and D. Returns whether or not values actually changed.
     */
    public boolean tune(double kP, double kI, double kD) {
        return tune(new PIDConstants(kP, kI, kD));
    }

    /**
     * Gets the current PID
     */
    public PIDConstants getConstants() {
        return pidConstants;
    }

    /**
     * Cycles this controller, using the current offset and time to adjust this controller as necessary.
     * The resulting output of this controller is returned.
     * If the controller detects that the elapsed time exceeds the timeout time, it will reset itself.
     *
     * @param offset        the offset, current value - target value.
     * @param curTimeMillis the time in milliseconds.
     */
    double lastOutput;
    public double loop(double offset, long curTimeMillis) {
        double error = -offset;
        int dT = (int) (curTimeMillis - prevTimeMillis);
        double dError = 0;
        if (dT > timeoutMillis) {
            integral += error;
            dError = (error - prevError);
            lastOutput = (pidConstants.kP * error) + (pidConstants.kI * integral) + (pidConstants.kD * dError);
            prevTimeMillis = curTimeMillis;
            prevError = error;
        }
        return lastOutput;
    }

    /**
     * Resets the integral sum as well as the previous error and time trackers.
     */
    public void reset() {
        integral = 0;
        prevError = 0;
        prevTimeMillis = 0;
    }

    /**
     * Resets the integral sum.
     */
    public void resetIntegral() {
        integral = 0;
    }
}