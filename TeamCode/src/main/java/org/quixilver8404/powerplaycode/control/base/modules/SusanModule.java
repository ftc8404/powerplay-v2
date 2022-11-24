package org.quixilver8404.powerplaycode.control.base.modules;

import org.quixilver8404.powerplaycode.control.base.HardwareCollection;
import org.quixilver8404.powerplaycode.control.base.Robot;

public class SusanModule {
    public double power;
    public double position;
    public double desiredpos;
    public double leftbound;
    public double rightbound;
    public double tolerance;
    public HardwareCollection hardwareMap;

    public SusanModule(HardwareCollection hardwareCollection) {
        //power can only be between 0 and 1
        power = 0.7;
        position = 0;
        desiredpos = 0;
        leftbound = 10000;
        rightbound = -10000;
        tolerance = 100;
        hardwareMap = hardwareCollection;
    }

    public synchronized void update() {
        position = hardwareMap.susanMotor1.getEncoderPosition();
    }

    public synchronized void powerMotor(double scalar){
        if (position > leftbound && position < rightbound){
            hardwareMap.slidesMotor1.setPower(scalar * power);
            hardwareMap.slidesMotor2.setPower(scalar * power);
        } else {
            hardwareMap.slidesMotor1.setPower(0);
            hardwareMap.slidesMotor2.setPower(0);
        }
    }
}
