package org.quixilver8404.powerplaycode.control.base.modules;

import org.quixilver8404.powerplaycode.control.base.HardwareCollection;
import org.quixilver8404.powerplaycode.control.base.Robot;

public class SlideModule {
    public double turnpower;
    public double position;
    public double desiredpos;
    public double upperbound;
    public double lowerbound;
    public double tolerance;

    public SlideModule(HardwareCollection hardwareCollection) {
        //turnpower can only be between 0 and 1
        turnpower = 0.7;
        position = 0;
        desiredpos = 0;
        upperbound = 13000;
        lowerbound = -10000;
        tolerance = 200;
    }

    public synchronized void update() {

    }

    public synchronized void setPower(){
        if (position > lowerbound && position < upperbound){

        }
    }
}
