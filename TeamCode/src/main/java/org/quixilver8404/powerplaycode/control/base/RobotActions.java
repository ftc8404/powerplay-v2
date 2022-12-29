package org.quixilver8404.powerplaycode.control.base;

import org.quixilver8404.breakout.feedforward.ActionEventListener;
import org.quixilver8404.powerplaycode.actions.RobotActionFunction;
import org.quixilver8404.powerplaycode.util.Vector3;

import java.util.ArrayList;
import java.util.Timer;
import java.util.TimerTask;

public class RobotActions extends ArrayList<ActionEventListener> {
    final double inToM = 0.0254;

    public RobotActions(final Robot robot) {
        add(new ActionEventListener(0, new RobotActionFunction(robot) {
            @Override
            public void run() {
                robot.breakoutModule.stop();
                robot.autoPilotModule.setDesiredPosition(new Vector3(66.58*inToM, 94.00*inToM, -Math.PI/2));
//                robot.slideModule.setDesiredpos(100);
//                robot.susanModule.setDesiredpos(578);
                final Timer timer = new Timer();
                timer.schedule(new TimerTask() {
                    @Override
                    public void run() {
                        try {
                            robot.autoPilotModule.disable();
//                            robot.slideModule.setDesiredpos(1030);
                            Thread.sleep(500);
                            robot.slideModule.goToJunc4();
                            Thread.sleep(500);
                            robot.susanModule.goToCustom(2000);
                            Thread.sleep(500);
                            robot.clawModule.setOpen();
                            Thread.sleep(200);
                            robot.clawModule.setClose();
                            robot.susanModule.goToFront();
                            robot.slideModule.goToGround();
//                            robot.slideModule.setDesiredpos(100);
                            Thread.sleep(500);
                            robot.breakoutModule.resume();
                        } catch (InterruptedException e) {
                            e.printStackTrace();
                        }
                    }
                }, 700);
            }
        }));
        add(new ActionEventListener(1, new RobotActionFunction(robot) {
            @Override
            public void run() {
                robot.breakoutModule.stop();
                if ("red".equals("red")) { //CV
                    robot.autoPilotModule.setDesiredPosition(new Vector3(11*inToM, 82.42*inToM, -Math.PI/2));
                } else if("blue".equals("blue")){ //CV
                    robot.autoPilotModule.setDesiredPosition(new Vector3(60*inToM, 82.42*inToM, -Math.PI/2));
                }
                robot.autoPilotModule.enable();
                final Timer timer = new Timer();
                timer.schedule(new TimerTask() {
                    @Override
                    public void run() {
                        robot.autoPilotModule.disable();
                        robot.breakoutModule.resume();
                    }
                }, 2000);
            }
        }));
        add(new ActionEventListener(2, new RobotActionFunction(robot) {
            @Override
            public void run() {
                robot.breakoutModule.stop();
                robot.autoPilotModule.setDesiredPosition(new Vector3(74.42*inToM, 94.00*inToM, -Math.PI/2));
//                robot.slideModule.setDesiredpos(100);
//                robot.susanModule.setDesiredpos(578);
                final Timer timer = new Timer();
                timer.schedule(new TimerTask() {
                    @Override
                    public void run() {
                        try {
                            robot.autoPilotModule.disable();
//                            robot.slideModule.setDesiredpos(1030);
                            Thread.sleep(500);
                            robot.slideModule.goToJunc4();
                            Thread.sleep(500);
                            robot.susanModule.goToCustom(-2000);
                            Thread.sleep(500);
                            robot.clawModule.setOpen();
                            Thread.sleep(200);
                            robot.clawModule.setClose();
                            robot.susanModule.goToFront();
                            robot.slideModule.goToGround();
//                            robot.slideModule.setDesiredpos(100);
                            Thread.sleep(500);
                            robot.breakoutModule.resume();
                        } catch (InterruptedException e) {
                            e.printStackTrace();
                        }
                    }
                }, 700);
            }
        }));
        add(new ActionEventListener(3, new RobotActionFunction(robot) {
            @Override
            public void run() {
                robot.breakoutModule.stop();

                robot.clawModule.setOpen();
                robot.autoPilotModule.setDesiredPosition(new Vector3(58.75*inToM,129.25*inToM,0));
                try {
                    Thread.sleep(1500);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }


            }
        }));
        //drops initial cone, left side auton
        add(new ActionEventListener(4, new RobotActionFunction(robot) {
            @Override
            public void run() {
                robot.breakoutModule.stop();
                robot.autoPilotModule.setDesiredPosition(new Vector3(62.67*inToM,94*inToM,0));
                try {
                    Thread.sleep(1500);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
                robot.slideModule.goToJunc4();
                try {
                    Thread.sleep(1000);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
                robot.susanModule.goToCustom(1533);
                try {
                    Thread.sleep(700);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
                robot.clawModule.setOpen();


            }
        }));
        //drops initial cone, right side auton
        add(new ActionEventListener(5, new RobotActionFunction(robot) {
            @Override
            public void run() {
                robot.breakoutModule.stop();
                robot.autoPilotModule.setDesiredPosition(new Vector3(62.67,94,0)); //change
                try {
                    Thread.sleep(1500);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
                robot.slideModule.goToJunc4();
                try {
                    Thread.sleep(1000);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
                robot.susanModule.goToCustom(-1533);
                robot.clawModule.setOpen();


            }
        }));
    }
}
