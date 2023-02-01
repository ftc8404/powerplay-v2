package org.quixilver8404.powerplay.control;

import org.quixilver8404.powerplay.util.Tunable;

public class ClawModule {
    @Tunable
    private static final double OPEN_GEAR = 0.6; // TODO tune
    @Tunable
    private static final double CLOSE_GEAR = 0.8; // TODO tune

    private ClawState clawState;

    public enum ClawState {
        OPEN, CLOSE, MOVING
    }

    public ClawModule() {
        clawState = ClawState.MOVING;
    }

    public synchronized void update(HardwareCollection hardwareCollection) {
        switch (clawState) {
            case OPEN:
                clawState = ClawState.MOVING;
                hardwareCollection.gearServo.setPosition(OPEN_GEAR);
                clawState = ClawState.OPEN;
                break;
            case CLOSE:
                clawState = ClawState.MOVING;
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
