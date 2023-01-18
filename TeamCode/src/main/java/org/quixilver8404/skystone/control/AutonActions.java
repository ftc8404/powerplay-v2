package org.quixilver8404.skystone.control;

import org.quixilver8404.skystone.util.Tunable;
import org.quixilver8404.skystone.util.measurement.Distance;

public class AutonActions {

    // sets default velocities for intake or outtake powers
    @Tunable
    public static final double INTAKE_POWER = 0.8;
    @Tunable
    public static final double INTAKE_FAST_POWER = 1.0;
    @Tunable
    public static final double INTAKE_SLOW_POWER = 0.6;
    @Tunable
    public static final double OUTTAKE_POWER = -0.4;
    @Tunable
    public static final double LIFT_RAISE_HEIGHT_INCHES = 7.5;

    /**
     * 01: starts intaking
     * 02: starts outtaking (reverses intake)
     * 03: stops all intake actions
     * 04: close claw clamp
     * 05: open claw clamp
     * 06: sets the output to the IN_UP state
     * 07: sets the output to the IN_DOWN_OPEN state
     * 08: sets the output to the IN_DOWN_CLOSED state
     * 09: sets the output to the PLACE_STONE state
     * 10: stone release sequence
     * 11: sets the output to the IN_UP state with a delay
     * 15: lowers lift
     * 16: raises lift
     * 20: sets the foundation servo to down
     * 21: sets the foundation servo to up
     * 30: runs the intake at fast power
     * 31: run the intake at fast power for a short time
     * 32: runs the intake at a slower speed
     * 40: intake state sequence
     * 41: sets the output to the IN_DOWN_CLOSED_AUTON state
     * 100: to be run throughout 5-stone auton
     */
    public static void runAction(int actionID, BaseRobot baseRobot) {
        double height = 8;
        switch (actionID) {
            case 0:
                baseRobot.taskModule.addTask(new TaskModule.Task() {
                    @Override
                    public boolean loop(int runningTimeMillis, BaseRobot baseRobot) {
                        if (runningTimeMillis < 1000) {
                            baseRobot.slidesModule.setTargetPositionPreset(SlidesModule.SlidePositionPreset.JUNC_4);
                            baseRobot.susanModule.goToCustomDeg(90);
                        } else if (runningTimeMillis < 1200){
                            baseRobot.clawModule.setOpen();
                        } else {
                            baseRobot.susanModule.goToFront();
                            baseRobot.clawModule.setClose();
                            baseRobot.slidesModule.setTargetPositionPreset(SlidesModule.SlidePositionPreset.GROUND);
                        }
                        return true;
                    }
                });
                break;
            case 1:
                double finalHeight = height;
                baseRobot.taskModule.addTask(new TaskModule.Task() {
                    @Override
                    public boolean loop(int runningTimeMillis, BaseRobot baseRobot) {
                        if (runningTimeMillis < 500) {
                            baseRobot.slidesModule.setTargetPosition(new Distance(finalHeight, Distance.Unit.INCHES));
                            baseRobot.clawModule.setOpen();
                        } else if (runningTimeMillis < 700){
                            baseRobot.clawModule.setClose();
                        } else {
                            baseRobot.slidesModule.setTargetPositionPreset(SlidesModule.SlidePositionPreset.JUNC_1);
                        }
                        return true;
                    }
                });
                height -= 1.5;
                break;
//            case 1:
//                baseRobot.intakeModule.setTargetPowers(INTAKE_POWER, INTAKE_POWER);
//                break;
//            case 2:
//                baseRobot.intakeModule.setTargetPowers(OUTTAKE_POWER, OUTTAKE_POWER);
//                break;
//            case 3:
//                baseRobot.intakeModule.setTargetPowers(0, 0);
//                break;
//            case 4:
//                baseRobot.outputModule.outputStateModule.setPlaceClampClosed();
//                break;
//            case 5:
//                baseRobot.outputModule.outputStateModule.setPlaceClampOpen();
//                break;
//            case 6:
//                baseRobot.outputModule.outputStateModule.setOutputState(OutputState.IN_UP, baseRobot);
//                break;
//            case 7:
//                baseRobot.outputModule.outputStateModule.setOutputState(OutputState.IN_DOWN_OPEN, baseRobot);
//                break;
//            case 8:
//                baseRobot.outputModule.outputStateModule.setOutputState(OutputState.IN_DOWN_CLOSED, baseRobot);
//                break;
//            case 9:
//                baseRobot.outputModule.outputStateModule.setOutputState(OutputState.PLACE_STONE_MID, baseRobot);
//                break;
//            case 10:
//                baseRobot.taskModule.addTask(new TaskModule.Task() {
//                    @Override
//                    public boolean loop(int runningTimeMillis, BaseRobot baseRobot) {
//                        if (runningTimeMillis < 450) {
//                            baseRobot.outputModule.outputStateModule.setOutputState(OutputState.PLACE_STONE_MID_AUTON, baseRobot);
//                        } else if (runningTimeMillis < 600) {
//                            baseRobot.outputModule.outputStateModule.setOutputState(OutputState.PLACE_STONE_MID_AUTON, baseRobot);
//                            baseRobot.outputModule.outputStateModule.setPlaceClampOpen();
//                        } else {
//                            baseRobot.outputModule.outputStateModule.setOutputState(OutputState.RELEASED_STONE_AUTON, baseRobot);
//                            return true;
//                        }
//                        return false;
//                    }
//                });
//                break;
//            case 11:
//                baseRobot.taskModule.addTask(new TaskModule.Task() {
//                    @Override
//                    public boolean loop(int runningTimeMillis, BaseRobot baseRobot) {
//                        if (runningTimeMillis < 300) {
//                            return false;
//                        } else {
//                            baseRobot.outputModule.outputStateModule.setOutputState(OutputState.IN_UP, baseRobot);
//                            return true;
//                        }
//                    }
//                });
//                break;
//            case 15:
//                baseRobot.outputModule.liftModule.setTargetPosition(Distance.ZERO);
//                break;
//            case 16:
//                baseRobot.outputModule.liftModule.setTargetPosition(new Distance(LIFT_RAISE_HEIGHT_INCHES, Distance.Unit.INCHES));
//                break;
//            case 20:
//                baseRobot.foundationModule.setTargetServosDown();
//                break;
//            case 21:
//                baseRobot.foundationModule.setTargetServosUp();
//                break;
//            case 30:
//                baseRobot.intakeModule.setTargetPowers(INTAKE_FAST_POWER, INTAKE_FAST_POWER);
//                break;
//            case 31:
//                baseRobot.taskModule.addTask(new TaskModule.Task() {
//                    @Override
//                    public boolean loop(int runningTimeMillis, BaseRobot baseRobot) {
//                        if (runningTimeMillis < 300) {
//                            baseRobot.intakeModule.setTargetPowers(INTAKE_FAST_POWER, INTAKE_FAST_POWER);
//                            return false;
//                        } else {
//                            baseRobot.intakeModule.setTargetPowers(0, 0);
//                            return true;
//                        }
//                    }
//                });
//                break;
//            case 32:
//                baseRobot.intakeModule.setTargetPowers(INTAKE_SLOW_POWER, INTAKE_SLOW_POWER);
//                break;
//            case 40:
//                baseRobot.taskModule.addTask(new TaskModule.Task() {
//                    @Override
//                    public boolean loop(int runningTimeMillis, BaseRobot baseRobot) {
//                        if (baseRobot.outputModule.outputStateModule.getOutputState() == OutputState.IN_DOWN_CLOSED_AUTON) {
//                            baseRobot.intakeModule.setTargetPowers(OUTTAKE_POWER, OUTTAKE_POWER);
//                            return true;
//                        }
//
//                        if (runningTimeMillis < 800) {
//                            baseRobot.intakeModule.setTargetPowers(INTAKE_FAST_POWER, INTAKE_FAST_POWER);
//                            baseRobot.outputModule.outputStateModule.setOutputState(OutputState.IN_UP, baseRobot);
//                        } else if (runningTimeMillis < 1600) {
//                            baseRobot.intakeModule.setTargetPowers(INTAKE_FAST_POWER, INTAKE_FAST_POWER);
//                            baseRobot.outputModule.outputStateModule.setOutputState(OutputState.IN_DOWN_OPEN, baseRobot);
//                        } else {
//                            baseRobot.intakeModule.setTargetPowers(OUTTAKE_POWER, OUTTAKE_POWER);
//                            baseRobot.outputModule.outputStateModule.setOutputState(OutputState.IN_DOWN_CLOSED_AUTON, baseRobot);
//                            return true;
//                        }
//                        return false;
//                    }
//                });
//                baseRobot.taskModule.addTask(new TaskModule.Task() {
//                    @Override
//                    public boolean loop(int runningTimeMillis, BaseRobot baseRobot) {
//                        if (runningTimeMillis > 2500) {
//                            baseRobot.intakeModule.setTargetPowers(0, 0);
//                            return true;
//                        }
//                        return false;
//                    }
//                });
//                break;
//            case 41:
//                baseRobot.outputModule.outputStateModule.setOutputState(OutputState.IN_DOWN_CLOSED_AUTON, baseRobot);
//                break;
//            case 100:
//                baseRobot.taskModule.addTask(new TaskModule.Task() {
//
//                    @Override
//                    public boolean loop(int runningTimeMillis, BaseRobot baseRobot) {
//                        // making sure the robot parks in time
//                        if (runningTimeMillis > 29.0 * 1000) {
//                            int minTVal = baseRobot.pathFollowModule.getTValOfAction(102);
//                            baseRobot.pathFollowModule.setMinLookaheadWindowStartInches(baseRobot.pathFollowModule.getDistAlongPath(minTVal) + 30.0);
//                        } else if (runningTimeMillis > 26.5 * 1000) {
//                            int minTVal = baseRobot.pathFollowModule.getTValOfAction(101);
//                            baseRobot.pathFollowModule.setMinLookaheadWindowStartInches(baseRobot.pathFollowModule.getDistAlongPath(minTVal) + 24.0);
//                        }
//                        return false;
//                    }
//                });
//                break;
        }
    }
}
