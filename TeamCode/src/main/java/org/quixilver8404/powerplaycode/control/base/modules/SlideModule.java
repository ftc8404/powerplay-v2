package org.quixilver8404.powerplaycode.control.base.modules;

import org.quixilver8404.powerplaycode.control.base.HardwareCollection;
import org.quixilver8404.powerplaycode.control.base.Robot;

public class SlideModule {
    public double power;
    public double position;
    public double desiredpos;
    public double upperbound;
    public double lowerbound;
    public double tolerance;
    public HardwareCollection hardwareMap;

    public SlideModule(HardwareCollection hardwareCollection) {
        //power can only be between 0 and 1
        power = 0.7;
        position = 0;
        desiredpos = 0;
        upperbound = 13000;
        lowerbound = -10000;
        hardwareMap = hardwareCollection;
    }

    public synchronized void update() {
        position = hardwareMap.slidesMotor1.getEncoderPosition();
    }

    public synchronized void powerMotor(double scalar){
        if (position > lowerbound && position < upperbound){
            hardwareMap.slidesMotor1.setPower(scalar * power);
            hardwareMap.slidesMotor2.setPower(scalar * power);
        } else {
            hardwareMap.slidesMotor1.setPower(0);
            hardwareMap.slidesMotor2.setPower(0);
        }
    }
}
