package org.quixilver8404.powerplaycode.control.base;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

/**
 * Calls robot.update() repeatedly on a separate thread until stopped
 */
public class HardwareLoopThread extends Thread {

    private final Robot robot;
    private final OpMode opMode;

    /**
     * Sets the robot and opmode equal to the respective inputed values
     * @param robot - references the actual robot itself
     * @param opMode - base class for user defined opmodes
     */
    public HardwareLoopThread(Robot robot, OpMode opMode) {
        super("hardwareLoopThread");
        this.robot = robot;
        this.opMode = opMode;
    }

    /**
     * Runs robot.update() continuously unless it is stopped or interrupted.
     */
    @Override
    public void run() {
        while (!isInterrupted()) {
            robot.update();
        }
    }
}
