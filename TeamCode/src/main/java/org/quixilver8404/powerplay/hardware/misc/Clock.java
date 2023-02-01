package org.quixilver8404.powerplay.hardware.misc;

import org.quixilver8404.powerplay.hardware.HardwareDevice;

/**
 * Provides a way to get the elapsed time since the robot's initialization or since the last update
 */
public class Clock extends HardwareDevice {
    private static final long initialDeltaTimeMillis = 10000000;
    private long startMillis;
    private long lastUpdateTimeMillis;
    private long deltaTimeMillis;
    private long runningTimeMillis;

    /**
     * Sets the running time to zero and the initial delta time to a very large number
     */
    public Clock(String name) {
        super(name);
        reset();
    }

    /**
     * Sets the running time to zero and the initial delta time to a very large number.
     * This sets the device name to "clock".
     */
    public Clock() {
        this("clock");
    }

    /**
     * Call this at the beginning of each iteration of the main robot loop. Subsequent calls to
     * getDeltaTimeMillis() will return the same value until update() is called again.
     * The first time update() is called, deltaTimeMillis will be a very large number rather than
     * zero to avoid arithmetic errors.
     */
    public synchronized void update() {
        runningTimeMillis = System.currentTimeMillis() - startMillis;
        if (lastUpdateTimeMillis != -1) {
            deltaTimeMillis = runningTimeMillis - lastUpdateTimeMillis;
        }
        lastUpdateTimeMillis = runningTimeMillis;
    }

    /**
     * Gets the delta time between the last two calls to update()
     */
    public synchronized int getDeltaTimeMillis() {
        return (int) deltaTimeMillis;
    }

    /**
     * Gets the time since the clock's creation or since reset()
     */
    public synchronized int getRunningTimeMillis() {
        return (int) runningTimeMillis;
    }

    /**
     * Resets the running time and makes the next call to getDeltaTimeMillis() return a very
     * large number until two calls to update() are made
     */
    public synchronized void reset() {
        startMillis = System.currentTimeMillis();
        lastUpdateTimeMillis = -1;
        deltaTimeMillis = initialDeltaTimeMillis;
    }
}
