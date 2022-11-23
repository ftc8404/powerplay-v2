package org.quixilver8404.powerplaycode.control.base.modules;

import org.quixilver8404.powerplaycode.control.base.HardwareCollection;
import org.quixilver8404.powerplaycode.control.base.Robot;

public class SusanModule {
    double turnpower;
    double position;
    double desiredpos;
    double leftbound;
    double rightbound;
    double tolerance;

    public SusanModule() {
        //turnpower can only be between 0 and 1
        turnpower = 0.7;
        position = 0;
        desiredpos = 0;
        leftbound = 10000;
        rightbound = -10000;
        tolerance = 100;
    }

    public synchronized void update(HardwareCollection hardwareCollection) {
        // updates position, finds average of the encoder readings
        position = (hardwareCollection.susanMotor1.getEncoderPosition()
                + hardwareCollection.susanMotor2.getEncoderPosition())/2;

        // moves turret to desired position
        if (desiredpos - position >= tolerance){
            hardwareCollection.susanMotor1.setPower(turnpower);
            hardwareCollection.susanMotor2.setPower(turnpower);
        } else if (desiredpos - position <= -tolerance){
            hardwareCollection.susanMotor1.setPower(-turnpower);
            hardwareCollection.susanMotor2.setPower(-turnpower);
        } else if (Math.abs(desiredpos - position) <= tolerance){
            hardwareCollection.susanMotor1.setPower(0);
            hardwareCollection.susanMotor2.setPower(0);
        }
    }

    public synchronized void setTurnpower(double turnpower) {
        this.turnpower = turnpower;
    }

    public synchronized void resetpos() {
        this.position = 0;
    }

    public synchronized void turnturret(double distance) {
        desiredpos += distance;
        if (desiredpos > leftbound){
            desiredpos = leftbound;
        } else if (desiredpos < rightbound){
            desiredpos = rightbound;
        }
    }

    public synchronized void setDesiredpos(double desiredpos){
        this.desiredpos = desiredpos;
    }
}
