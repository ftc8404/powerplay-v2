package org.quixilver8404.powerplay.control;

import android.util.Log;

import org.quixilver8404.powerplay.util.Tunable;

public class SusanModule {

    public enum SusanControlState {
        GO_TO_FRONT, GO_TO_CUSTOM, MANUAL
    }

    @Tunable
    public static final double RIGHT_BOUND_DEG = 200;
    @Tunable
    public static final double LEFT_BOUND_DEG = -200;
    @Tunable
    public static final double GO_TO_TOLERANCE_DEG = 5;

    @Tunable
    // counts-per-rev / 360 deg-per-rev
    public static final double COUNTS_PER_DEG = 5544 / 360.0;

    @Tunable
    public static final double KP = 0.052; // TODO tune these
    @Tunable
    public static final double KI = 0; // TODO tune these
    @Tunable
    public static final double KD = 0; // TODO tune these


    private SusanControlState susanControlState;

    private final PIDController pidController;

    private BaseRobot robot;

    private double curPosDeg;
    private double targetPower;
    private double targetPosDeg;

    public SusanModule(BaseRobot robot) {
        susanControlState = SusanControlState.MANUAL;
        curPosDeg = 0;
        pidController = new PIDController(KP, KI, KD, 0);
        this.robot = robot;
    }

    public synchronized void update(final SlidesModule slideModule, HardwareCollection hwCollection) {
        double counts = hwCollection.susanMotor1.getEncoder().getEncoderPosition();
        curPosDeg = counts / COUNTS_PER_DEG;

        double desiredPow;

        if (susanControlState == SusanControlState.MANUAL) {
            desiredPow = targetPower;
            pidController.reset();
        } else {
            double desiredPos = susanControlState == SusanControlState.GO_TO_FRONT
                    ? 0 : targetPosDeg;

            desiredPow = pidController.loop(curPosDeg - desiredPos, hwCollection.clock.getRunningTimeMillis());
            System.out.println("desiredPow: " + desiredPow);
        }

        if ((slideModule.areSlidesLowered() && susanControlState != SusanControlState.GO_TO_FRONT)
                || (curPosDeg >= RIGHT_BOUND_DEG && desiredPow > 0)
                || (curPosDeg <= LEFT_BOUND_DEG && desiredPow < 0)) {
            desiredPow = 0;
            pidController.reset();
        }

        if (Math.abs(desiredPow) > 1) {
            pidController.resetIntegral();
        }

        Log.d("TurretModule", "power: " + desiredPow + "  delta_t: " + hwCollection.clock.getDeltaTimeMillis() + "  error: " + (curPosDeg - (susanControlState == SusanControlState.GO_TO_FRONT ? 0 : targetPosDeg)));
        hwCollection.susanMotor1.setPower(desiredPow);
        hwCollection.susanMotor2.setPower(desiredPow);
    }

    public synchronized boolean isSafeToLowerSlides() {
//        if (susanControlState != SusanControlState.GO_TO_FRONT) {
//            return false;
//        }

        return Math.abs(curPosDeg) <= GO_TO_TOLERANCE_DEG;
    }

    public synchronized SusanControlState getSusanControlState() {
        return susanControlState;
    }

    public synchronized void goToFront() {
        susanControlState = SusanControlState.GO_TO_FRONT;
    }

    public synchronized void goToCustomDeg(double customPosDeg) {
        this.targetPosDeg = customPosDeg;
        susanControlState = SusanControlState.GO_TO_CUSTOM;
    }

    public synchronized void goToPreloadedCone() {
        this.targetPosDeg = -2088 / COUNTS_PER_DEG;
        susanControlState = SusanControlState.GO_TO_CUSTOM;
    }

    public synchronized void setManualPower(double power) {
        this.targetPower = power;
        if (power != 0){
            susanControlState = SusanControlState.MANUAL;
        }
    }
    public synchronized boolean isMoving(){
        return robot.hwCollection.susanMotor1.getPower() <= 0.005;
    }
    public synchronized double getCurPosDeg() {
        return curPosDeg;
    }
}
