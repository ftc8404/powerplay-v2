package org.quixilver8404.skystone.control;

import org.quixilver8404.skystone.util.Tunable;

public class ClawModule {
    @Tunable
    private static final double OPEN_GEAR = 0.00; // TODO tune
    @Tunable
    private static final double CLOSE_GEAR = 1.00; // TODO tune

    private ClawState clawState;

    public enum ClawState {
        OPEN, CLOSE
    }

    public ClawModule() {
        clawState = ClawState.OPEN;
    }

    public synchronized void update(HardwareCollection hardwareCollection) {
        switch (clawState) {
            case OPEN:
                hardwareCollection.gearServo.setPosition(OPEN_GEAR);
                clawState = ClawState.OPEN;
                break;
            case CLOSE:
                hardwareCollection.gearServo.setPosition(CLOSE_GEAR);
                clawState = ClawState.CLOSE;
                break;
        }
    }

    public synchronized void setOpen() {
        clawState = ClawState.OPEN;
    }

    public synchronized void setClose() {
        clawState = ClawState.CLOSE;
    }

    public synchronized String getClawState() {
        if (clawState == ClawState.OPEN) {
            return "Open";
        } else {
            return "Close";
        }
    }
}
