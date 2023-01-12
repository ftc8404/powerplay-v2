package org.quixilver8404.skystone.control;

import org.quixilver8404.skystone.util.Tunable;

public class SusanModule {

    public enum SusanControlState {
        GO_TO_FRONT, GO_TO_CUSTOM, MANUAL
    }

    @Tunable
    public static final double RIGHT_BOUND = 2700;
    @Tunable
    public static final double LEFT_BOUND = -2700;
    @Tunable
    public static final double GO_TO_TOLERANCE = 80;
    @Tunable
    public static final double FRONT_POS = 0;

    @Tunable
    public static final double KP = 1; // TODO tune these
    @Tunable
    public static final double KI = 0; // TODO tune these
    @Tunable
    public static final double KD = 0; // TODO tune these


    protected SusanControlState susanControlState;

    protected final PIDController pidController;

    public double curPosition;
    public double targetPower;
    public double targetPosition;

    public SusanModule() {
        susanControlState = SusanControlState.MANUAL;
        curPosition = 0;
        pidController = new PIDController(KP, KI, KD, 0);
    }

    public synchronized void update(final SlidesModule slideModule, HardwareCollection hwCollection) {
        curPosition = hwCollection.susanMotor1.getEncoder().getEncoderPosition();

        double desiredPow;

        if (susanControlState == SusanControlState.MANUAL) {
            desiredPow = Math.signum(targetPower) * Math.pow(targetPower, 2);
            pidController.reset();
        } else {
            double desiredPos = susanControlState == SusanControlState.GO_TO_FRONT
                    ? FRONT_POS : targetPosition;

            desiredPow = pidController.loop(curPosition - desiredPos, hwCollection.clock.getDeltaTimeMillis());
        }

        if ((slideModule.areSlidesLowered() && susanControlState != SusanControlState.GO_TO_FRONT)
                || (curPosition >= RIGHT_BOUND && desiredPow > 0)
                || (curPosition <= LEFT_BOUND && desiredPow < 0)) {
            desiredPow = 0;
            pidController.reset();
        }

        if (Math.abs(desiredPow) > 1) {
            pidController.resetIntegral();
        }

        hwCollection.susanMotor1.setPower(desiredPow);
        hwCollection.susanMotor2.setPower(desiredPow);
    }

    public synchronized boolean isSafeToLowerSlides() {
        // uncomment below if we want to only let slides be lowered when using GO_TO_FRONT state
        // (i.e., don't allow slides to be lowered when manually controlling power, even if centered)

//        if (susanControlState != SusanControlState.GO_TO_FRONT) {
//            return false;
//        }

        return Math.abs(curPosition - FRONT_POS) <= GO_TO_TOLERANCE;
    }

    public synchronized SusanControlState getSusanControlState() {
        return susanControlState;
    }

    public synchronized void goToFront() {
        susanControlState = SusanControlState.GO_TO_FRONT;
    }

    public synchronized void goToCustom(double customPos) {
        this.targetPosition = customPos;
        susanControlState = SusanControlState.GO_TO_CUSTOM;
    }

    public synchronized void setManualPower(double power) {
        this.targetPower = power;
        susanControlState = SusanControlState.MANUAL;
    }
}
