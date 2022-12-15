package org.quixilver8404.powerplaycode.control.base.modules;

import android.util.Log;

import org.quixilver8404.powerplaycode.control.algorithms.PIDController;
import org.quixilver8404.powerplaycode.control.base.HardwareCollection;
import org.quixilver8404.powerplaycode.control.base.Robot;
import org.quixilver8404.powerplaycode.hardware.motors.EncoderMotor;

public class SusanModule {

    public enum SusanState {
        FRONT, AT_CUSTOM, IN_MOTION;
    }
    public enum SusanAction {
        GO_TO_FRONT, GO_TO_CUSTOM, NOT_MOVING;
    }

    public enum SusanControlState {
        MANUAL, AUTO;
    }

    public static final double SCALAR = 1.0;
    public static final double RIGHT_BOUND = 2300;
    public static final double LEFT_BOUND = -2300;
    public static final double TOLERANCE = 80;
    public static final double FRONT_POS = 0;

    public static final double KP = 1;
    public static final double KI = 0;
    public static final double KD = 0;
    public static final double KF = 0;

    public static final double ENCODER_PER_REV = 20 * 28 * 99/10d;

    protected SusanState susanState;
    protected SusanAction susanAction;

    protected SlideModule.SlideState slideState;

    protected SusanControlState susanControlState;

    protected final PIDController pidController;

    protected final EncoderMotor susanMotor1;
    protected final EncoderMotor susanMotor2;

    public double position;
    public double power;
    public double customPos;

    public SusanModule(final Robot robot, final HardwareCollection hardwareCollection, final SlideModule.SlideState slideState) {
        //power can only be between 0 and 1
        susanState  = SusanState.FRONT;
        susanAction = SusanAction.NOT_MOVING;
        susanControlState = SusanControlState.MANUAL;
        susanMotor1 = hardwareCollection.susanMotor1;
        susanMotor2 = hardwareCollection.susanMotor2;
        position = 0;
        pidController = new PIDController(KP, KI, KD);
        this.slideState = slideState;
    }

    public synchronized void update(final Robot robot) {
        position = susanMotor1.getEncoderPosition();
        if (Math.abs(position - FRONT_POS) <= TOLERANCE && (susanAction == SusanAction.GO_TO_FRONT || susanAction == SusanAction.NOT_MOVING)) {
            susanState = SusanState.FRONT;
        } else if (Math.abs(position - customPos) <= TOLERANCE) {
            susanState = SusanState.AT_CUSTOM;
        } else {
            susanState = SusanState.IN_MOTION;
        }
        slideState = robot.slideModule.getSlideState();
        double desiredPow = 0;
        if (susanControlState == SusanControlState.MANUAL){
            if (slideState == SlideModule.SlideState.GROUND || slideState == SlideModule.SlideState.IN_BETWEEN_BELOW_DRIVE) {
                desiredPow = 0;
            } else {
                desiredPow = Math.signum(power)*Math.pow(power,2);
            }
        } else {
            double desiredPos = Double.NaN;
            switch (susanAction) {
                case GO_TO_FRONT:
                    desiredPos = FRONT_POS;
                    break;
                case GO_TO_CUSTOM:
                    desiredPos = customPos;
                    break;
            }
            if (!Double.isNaN(desiredPos)){
                desiredPow = pidController.update(((desiredPos-position)/ENCODER_PER_REV)*2*Math.PI, robot.hardwareCollection.clock.getDeltaTimeMS()/1000d);
            } else {
                desiredPow = 0;
            }
        }
        if (position >= RIGHT_BOUND && desiredPow > 0){
            desiredPow = 0;
        }
        if (position <= LEFT_BOUND && desiredPow < 0){
            desiredPow = 0;
        }
        susanMotor1.setPower(desiredPow);
        susanMotor2.setPower(desiredPow);
    }

    public SusanState getSusanState() {
        return susanState;
    }
    public SusanAction getSusanAction() {
        return susanAction;
    }
    public SusanControlState getSusanControl() {
        return susanControlState;
    }
    public void goToFront() {
        susanAction = SusanAction.GO_TO_FRONT;
        susanControlState = SusanControlState.AUTO;
        pidController.reset();

    }
    public void goToCustom(double customPos) {
        this.customPos = customPos;
        susanAction = SusanAction.GO_TO_CUSTOM;
        susanControlState = SusanControlState.AUTO;
        pidController.reset();

    }
    public void setManualPower(double power) {
        this.power = power;
        susanControlState = SusanControlState.MANUAL;
        pidController.reset();
    }

//    public synchronized void powerMotor(double scalar){
//        if (position < leftbound && position > rightbound){
//            hardwareMap.susanMotor1.setPower(scalar * power);
//            hardwareMap.susanMotor2.setPower(scalar * power);
//        } else {
//            hardwareMap.susanMotor1.setPower(0);
//            hardwareMap.susanMotor2.setPower(0);
//        }
//    }
}
