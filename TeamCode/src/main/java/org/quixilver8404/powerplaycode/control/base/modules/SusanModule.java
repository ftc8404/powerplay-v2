package org.quixilver8404.powerplaycode.control.base.modules;

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
    public static final double RIGHT_BOUND = 2536;
    public static final double LEFT_BOUND = -10000;
    public static final double TOLERANCE = 40;
    public static final double FRONT_POS = 0;

    public static final double KP = 1;
    public static final double KI = 1;
    public static final double KD = 1;
    public static final double KF = 0;

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
        if (susanControlState == SusanControlState.MANUAL){
            if (slideState == SlideModule.SlideState.GROUND || slideState == SlideModule.SlideState.IN_BETWEEN_BELOW_DRIVE) {
                susanMotor1.setPower(0);
                susanMotor2.setPower(0);
            } else {
                susanMotor1.setPower(Math.signum(power)*Math.pow(power,2));
                susanMotor2.setPower(Math.signum(power)*Math.pow(power,2));
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
            double autoPower;
            if (desiredPos != Double.NaN){
                autoPower = pidController.update(desiredPos-position, robot.hardwareCollection.clock.getDeltaTimeMS()/1000d);
                susanMotor1.setPower(autoPower);
                susanMotor2.setPower(autoPower);
            } else {
                susanMotor1.setPower(0);
                susanMotor2.setPower(0);
            }
        }
    }

    public SusanState getSusanState() {
        return susanState;
    }
    public SusanAction getSusanAction() {
        return susanAction;
    }
    public void goToFront() {
        susanAction = SusanAction.GO_TO_FRONT;
    }
    public void goToCustom(double customPos) {
        this.customPos = customPos;
        susanAction = SusanAction.GO_TO_CUSTOM;
    }
    public void setManualPower(double power) {
        this.power = power;
    }
    public void setAutoMode(){
        susanControlState = SusanControlState.AUTO;
    }
    public void setManualMode(){
        susanControlState = SusanControlState.MANUAL;
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
