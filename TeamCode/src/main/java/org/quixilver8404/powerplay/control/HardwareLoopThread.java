package org.quixilver8404.powerplay.control;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

/**
 * Calls robot.update() repeatedly on a separate thread until stopped
 */
public class HardwareLoopThread extends Thread {

    private volatile boolean terminateRequested = false;
    private final BaseRobot robot;
    private final LinearOpMode opMode;

    public HardwareLoopThread(BaseRobot robot, LinearOpMode opMode) {
        super("hardwareLoopThread");
        this.robot = robot;
        this.opMode = opMode;
    }

    @Override
    public void run() {
        terminateRequested = false;
        while (!terminateRequested && !opMode.isStopRequested()) {
            robot.update();
        }
    }

    public void terminate() {
        terminateRequested = true;
    }

    public boolean isTerminateRequested() {
        return terminateRequested;
    }
}
