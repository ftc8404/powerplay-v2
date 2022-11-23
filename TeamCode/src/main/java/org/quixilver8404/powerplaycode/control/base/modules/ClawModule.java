package org.quixilver8404.powerplaycode.control.base.modules;

import static org.quixilver8404.powerplaycode.control.base.modules.ClawModule.ClawState.CLOSE;
import static org.quixilver8404.powerplaycode.control.base.modules.ClawModule.ClawState.OPEN;
import static org.quixilver8404.powerplaycode.control.base.modules.ClawModule.ClawState.MOVE;


import org.quixilver8404.powerplaycode.control.base.HardwareCollection;
import org.quixilver8404.powerplaycode.control.base.Robot;

public class ClawModule {
    private static final double OPEN_GEAR = 0.00;
    private static final double CLOSE_GEAR = 1.00;
    protected ClawState clawState;

    public enum ClawState {
        OPEN, CLOSE, MOVE
    }

    public ClawModule(){
        clawState = OPEN;

    }
    public synchronized void update(HardwareCollection hardwareCollection){
        switch (clawState){
            case OPEN:
                clawState = MOVE;
                hardwareCollection.gearServo.setPosition(OPEN_GEAR);
                clawState = OPEN;
                break;
            case CLOSE:
                clawState = MOVE;
                hardwareCollection.gearServo.setPosition(CLOSE_GEAR);
                clawState = CLOSE;
                break;
        }
    }

    public synchronized void setOpen(){
        clawState = OPEN;
    }

    public synchronized void setClose(){
        clawState = CLOSE;
    }

    public synchronized String getClawState(){
        if(clawState == OPEN){
            return "Open";
        } else if (clawState == MOVE){
            return "Move";
        } else {
            return "Close";
        }
    }
}
