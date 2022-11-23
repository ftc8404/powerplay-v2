package org.quixilver8404.powerplaycode.hardware.misc;

import org.quixilver8404.powerplaycode.hardware.HardwareDevice;

/**
 * Provides a way to get the elapsed time since the robot's initialization or since the last update
 */
public class Clock extends HardwareDevice {
    private static final long initialDeltaTimeMS = 10000000;
    private long startMS;
    private long lastUpdateTimeMS;
    private long deltaTimeMS;
    private long runningTimeMS;

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
     * getDeltaTimeMS() will return the same value until update() is called again.
     * The first time update() is called, deltaTimeMS will be a very large number rather than
     * zero to avoid arithmetic errors.
     */
    public synchronized void update() {
        runningTimeMS = System.currentTimeMillis() - startMS;
        if (lastUpdateTimeMS != -1) {
            deltaTimeMS = runningTimeMS - lastUpdateTimeMS;
        }
        lastUpdateTimeMS = runningTimeMS;
    }

    /**
     * Gets the delta time between the last two calls to update()
     */
    public synchronized long getDeltaTimeMS() {
        return deltaTimeMS;
    }

    /**
     * Gets the time since the clock's creation or since reset()
     */
    public synchronized long getRunningTimeMS() {
        return runningTimeMS;
    }

    /**
     * Resets the running time and makes the next call to getDeltaTimeMS() return a very
     * large number until two calls to update() are made
     */
    public synchronized void reset() {
        startMS = System.currentTimeMillis();
        lastUpdateTimeMS = -1;
        deltaTimeMS = initialDeltaTimeMS;
    }
}
