package org.quixilver8404.powerplaycode.actions;

import org.quixilver8404.breakout.feedforward.ActionEventListener;
import org.quixilver8404.breakout.feedforward.ActionFunction;
import org.quixilver8404.powerplaycode.control.base.Robot;

public class RobotActionEventListener extends ActionEventListener {

    protected final Robot robot;

    public RobotActionEventListener(final int action, final RobotActionFunction actionFunction, final Robot robot) {
        super(action, actionFunction);
        this.robot = robot;
    }
}
