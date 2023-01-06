package org.quixilver8404.skystone.control;

import org.quixilver8404.skystone.hardware.misc.Clock;

import java.util.LinkedList;

public class DiagnosticModule {

    private int updateCount = 0; // records the number of updates

    private int loopFrequencyHz;
    private final LinkedList<Integer> updateTimeHistoryMillis; // for tracking loop frequency

    public DiagnosticModule() {
        loopFrequencyHz = 0;
        updateTimeHistoryMillis = new LinkedList<>();
    }

    public synchronized void update(BaseRobot baseRobot, HardwareCollection hardwareCollection) {
        Clock clock = hardwareCollection.clock;

        updateCount++;

        // compute the running frequency
        int runningTimeMillis = clock.getRunningTimeMillis();
        updateTimeHistoryMillis.addFirst(runningTimeMillis);
        while (!updateTimeHistoryMillis.isEmpty() &&
                runningTimeMillis - updateTimeHistoryMillis.peekLast() >= 1000) {
            updateTimeHistoryMillis.removeLast();
        }
        loopFrequencyHz = updateTimeHistoryMillis.size();
    }

    /**
     * Returns the number of times update() was called
     */
    public synchronized int getUpdateCount() {
        return updateCount;
    }

    /**
     * Gets the frequency of the robot's loop in loops per second. This value is a running
     * count.
     */
    public synchronized int getLoopFrequencyHz() {
        return loopFrequencyHz;
    }
}
