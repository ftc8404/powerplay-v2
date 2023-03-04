package org.quixilver8404.powerplay.control;

import com.qualcomm.robotcore.robot.Robot;

import org.quixilver8404.powerplay.util.Tunable;

public class ClawModule {
    @Tunable
    private static final double OPEN_GEAR = 0.6; // TODO tune
    @Tunable
    private static final double CLOSE_GEAR = 0.8; // TODO tune

    private static final double CLOSE_ENCODER_DIFF = 1000;
    public static final double CONE_ENCODER_DIFF = 500;


    private ClawState clawState;
    BaseRobot robot;

    public enum ClawState {
        OPEN(0), CLOSE(900), MOVING(-1);
        public double clawCoder;
        ClawState(final double clawCoder){
            this.clawCoder = clawCoder;
        }
    }

    public ClawModule(BaseRobot robot) {
        clawState = ClawState.MOVING;
        this.robot = robot;
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
        } else if (clawState == ClawState.CLOSE){
            return "Close";
        } else {
            return "Moving";
        }
    }
    public synchronized void setClawCoderClose(){
        ClawState.CLOSE.clawCoder = robot.hwCollection.clawCoder.getEncoderPosition();
        ClawState.OPEN.clawCoder = ClawState.CLOSE.clawCoder - CLOSE_ENCODER_DIFF;
    }
    public synchronized void setClawCoderOpen(){
        ClawState.OPEN.clawCoder = robot.hwCollection.clawCoder.getEncoderPosition();
        ClawState.CLOSE.clawCoder = ClawState.OPEN.clawCoder + CLOSE_ENCODER_DIFF;
    }
}
