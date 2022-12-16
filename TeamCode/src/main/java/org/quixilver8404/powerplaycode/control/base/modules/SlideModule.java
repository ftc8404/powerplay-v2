package org.quixilver8404.powerplaycode.control.base.modules;

import android.util.Log;

import org.quixilver8404.powerplaycode.control.algorithms.PIDController;
import org.quixilver8404.powerplaycode.control.base.HardwareCollection;
import org.quixilver8404.powerplaycode.control.base.Robot;
import org.quixilver8404.powerplaycode.hardware.motors.EncoderMotor;

public class SlideModule {

    public enum SlideState {
        GROUND(-10), ABOVE_DRIVE(660), JUNC_1(1000), JUNC_2(2070), JUNC_3(3700),
        JUNC_4(6000), IN_BETWEEN_ABOVE_DRIVE(0), IN_BETWEEN_BELOW_DRIVE(0);

        public final double height;
        SlideState(final double height) {
            this.height = height;
        }
    }
    public enum SlideAction {
        GO_TO_GROUND, GO_TO_ABOVE_DRIVE, GO_TO_JUNC_1, GO_TO_JUNC_2, GO_TO_JUNC_3, GO_TO_JUNC_4, NOT_MOVING;
    }

    public enum SlideControlState {
        MANUAL, AUTO;
    }

    public static final double SCALAR = 1.0;
    public static final double UPPER_BOUND = 100000;
    public static final double LOWER_BOUND = 100;
    public static final double TOLERANCE = 40;

    public static final double KP = 9.2;
    public static final double KI = 0.03;
    public static final double KD = 0.7;
    public static final double KF = 0;

    public static final double ENCODER_PER_REV = 45 * 28; //you sukc

    protected SlideState slideState;
    protected SlideAction slideAction;

    protected SusanModule.SusanState susanState;

    protected SlideControlState slideControlState;

    protected final PIDController pidController;

    protected final EncoderMotor slideMotor1;
    protected final EncoderMotor slideMotor2;

    public double position;
    public double power;


    public SlideModule(final Robot robot, final HardwareCollection hardwareCollection) {
        //power can only be between 0 and 1
        slideState  = SlideState.GROUND;
        slideAction = SlideAction.NOT_MOVING;
        susanState = robot.susanModule.getSusanState();
        slideControlState = SlideControlState.MANUAL;
        slideMotor1 = hardwareCollection.slidesMotor1;
        slideMotor2 = hardwareCollection.slidesMotor2;
        position = 0;
        pidController = new PIDController(KP, KI, KD);

    }
    public synchronized void update(final Robot robot) {
        position = slideMotor1.getEncoderPosition();
        if (Math.abs(position - SlideState.JUNC_4.height) <= TOLERANCE){
            slideState = SlideState.JUNC_4;
        } else if (Math.abs(position - SlideState.JUNC_3.height) <= TOLERANCE){
            slideState = SlideState.JUNC_3;
        } else if (Math.abs(position - SlideState.JUNC_2.height) <= TOLERANCE){
            slideState = SlideState.JUNC_2;
        } else if (Math.abs(position - SlideState.JUNC_1.height) <= TOLERANCE){
            slideState = SlideState.JUNC_1;
        } else if (Math.abs(position - SlideState.ABOVE_DRIVE.height) <= TOLERANCE){
            slideState = SlideState.ABOVE_DRIVE;
        } else if (Math.abs(position - SlideState.GROUND.height) <= TOLERANCE){
            slideState = SlideState.GROUND;
        } else if (position > SlideState.ABOVE_DRIVE.height){
            slideState = SlideState.IN_BETWEEN_ABOVE_DRIVE;
        } else {
            slideState = SlideState.IN_BETWEEN_BELOW_DRIVE;
        }
        susanState = robot.susanModule.getSusanState();
        if (slideControlState == SlideControlState.MANUAL) {
            if (power >= 0 || position >= SlideState.ABOVE_DRIVE.height + TOLERANCE) {
                slideMotor2.setPower(Math.signum(power)*Math.pow(power,2));
                slideMotor1.setPower(Math.signum(power)*Math.pow(power,2));
            } else if (slideState == SlideState.ABOVE_DRIVE && susanState == SusanModule.SusanState.FRONT) {
                slideMotor2.setPower(Math.signum(power)*Math.pow(power,2));
                slideMotor1.setPower(Math.signum(power)*Math.pow(power,2));
            }
        } else {
            double desiredPos = Double.NaN;
            switch (slideAction) {
                case GO_TO_JUNC_4:
                    desiredPos = SlideState.JUNC_4.height;
                    break;
                case GO_TO_JUNC_3:
                    desiredPos = SlideState.JUNC_3.height;
                    break;
                case GO_TO_JUNC_2:
                    desiredPos = SlideState.JUNC_2.height;
                    break;
                case GO_TO_JUNC_1:
                    desiredPos = SlideState.JUNC_1.height;
                    break;
                case GO_TO_ABOVE_DRIVE:
                    desiredPos = SlideState.ABOVE_DRIVE.height;
                    break;
                case GO_TO_GROUND:
                    if (susanState == SusanModule.SusanState.FRONT) {
                        desiredPos = SlideState.GROUND.height;
                    }
                    break;
                case NOT_MOVING:
                    break;
            }
            double autoPower;
            if (!Double.isNaN(desiredPos)){
                autoPower = pidController.update(((desiredPos-position)/(ENCODER_PER_REV))*2.0*Math.PI*0.03, KF, robot.hardwareCollection.clock.getDeltaTimeMS()/1000d);
                Log.d("SlideModule", "Desired pow: " + autoPower + "  error: " + (desiredPos-position)/(ENCODER_PER_REV)*2.0*Math.PI*0.03 + "  error derivative: " + pidController.ddt_error + "  dt: " + robot.hardwareCollection.clock.getDeltaTimeMS()/1000d);

                slideMotor1.setPower(autoPower);
                slideMotor2.setPower(autoPower);
            } else {
                slideMotor1.setPower(0);
                slideMotor2.setPower(0);
            }
        }
    }
    public SlideState getSlideState() {
        return slideState;
    }
    public SlideAction getSlideAction() {
        return slideAction;
    }
    public SlideControlState getSlideControl() {
        return slideControlState;
    }
    public void goToJunc4() {
        slideAction = SlideAction.GO_TO_JUNC_4;
        slideControlState = SlideControlState.AUTO;
        pidController.reset();
    }
    public void goToJunc3() {
        slideAction = SlideAction.GO_TO_JUNC_3;
        slideControlState = SlideControlState.AUTO;
        pidController.reset();

    }
    public void goToJunc2() {
        slideAction = SlideAction.GO_TO_JUNC_2;
        slideControlState = SlideControlState.AUTO;
        pidController.reset();

    }
    public void goToJunc1() {
        slideAction = SlideAction.GO_TO_JUNC_1;
        slideControlState = SlideControlState.AUTO;
        pidController.reset();

    }
    public void goToAboveDrive() {
        slideAction = SlideAction.GO_TO_ABOVE_DRIVE;
        slideControlState = SlideControlState.AUTO;
        pidController.reset();

    }
    public void goToGround() {
        // can only do if facing forwards (claw)
        slideAction = SlideAction.GO_TO_GROUND;
        slideControlState = SlideControlState.AUTO;
        pidController.reset();

    }
    public void setManualPower(double power) {
        this.power = power;
        slideControlState = SlideControlState.MANUAL;
        pidController.reset();

    }


//    public synchronized void powerMotor(double power){
//        if (position > lowerbound && position < upperbound){
//            hardwareMap.slidesMotor1.setPower(Math.signum(power)*Math.pow(scalar * power,2));
//            hardwareMap.slidesMotor2.setPower(Math.signum(power)*Math.pow(scalar * power,2));
//        } else {
//            hardwareMap.slidesMotor1.setPower(0);
//            hardwareMap.slidesMotor2.setPower(0);
//        }
//    }
}
