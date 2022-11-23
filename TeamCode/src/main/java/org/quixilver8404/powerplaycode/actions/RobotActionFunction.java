package org.quixilver8404.powerplaycode.actions;

import org.quixilver8404.breakout.feedforward.ActionFunction;
import org.quixilver8404.powerplaycode.control.base.Robot;

public abstract class RobotActionFunction implements ActionFunction {

    public final Robot robot;
    public RobotActionFunction(final Robot robot) {
        this.robot = robot;
    }

}
