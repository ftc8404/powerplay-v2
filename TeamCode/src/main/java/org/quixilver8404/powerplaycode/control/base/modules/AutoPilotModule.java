package org.quixilver8404.powerplaycode.control.base.modules;

import org.quixilver8404.breakout.controller.AutoPilot;
import org.quixilver8404.breakout.util.Config;
import org.quixilver8404.powerplaycode.control.base.Robot;
import org.quixilver8404.powerplaycode.util.Vector3;

public class AutoPilotModule {

    public final Config config;
    public final AutoPilot autoPilot;

    protected boolean enabled;

    public AutoPilotModule(final Robot robot) {
        config = robot.config;
        autoPilot = new AutoPilot(config, true);
        enabled = false;
    }

    public void update(final Robot robot) {
        if (enabled) {
            robot.driveModule.setPowerCorrections(autoPilot.correction(robot.poseModule.getPos().toBreakoutVec(), robot.poseModule.getVel().toBreakoutVec()));
        } else {
            robot.driveModule.setPowerCorrections(new double[]{0,0,0,0});
        }
    }

    public synchronized void setDesiredPosition(final Vector3 pos) {
        autoPilot.setDesiredPos(pos.toBreakoutVec());
    }

    public synchronized void enable() {
        enabled = true;
    }

    public synchronized void disable() {
        enabled = false;
    }

    public synchronized boolean isEnabled() {
        return enabled;
    }
}
