package org.quixilver8404.powerplay.control;

import org.quixilver8404.powerplay.util.Tunable;
import org.quixilver8404.powerplay.util.measurement.Distance;

import java.util.Timer;
import java.util.TimerTask;

public class Actions {

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
    static double height = 4.5;
    boolean pickUpPreload = false;
    boolean dumpSignal = false;
    boolean dropCone = false;

    boolean coneStackPickup = false;

    boolean front = false;

    boolean open = false;

    int preloadStage = 0;
    double clawStartTimeMillis;

    double parkMillis;

    int signalStage = 0;

    int dropStage = 0;

    int frontStage = 0;

    int variant = 0;

    double liftHeight;

    int consPickStage = 0;

    int openStage = 0;


    boolean park;


    static BaseRobot robot;

    public Actions(BaseRobot robot) {
        Actions.robot = robot;
    }

    public synchronized void update() {
        boolean holdingCone = Math.abs(robot.hwCollection.clawCoder.getEncoderPosition() - (ClawModule.CONE_ENCODER_DIFF + ClawModule.ClawState.OPEN.clawCoder)) < 100
                && robot.clawModule.getClawState().equals("Close");
        boolean openCone = robot.clawModule.getClawState().equals("Open") && Math.abs(robot.hwCollection.clawCoder.getEncoderPosition() - ClawModule.ClawState.OPEN.clawCoder) < 10;
        if (dumpSignal) {
            if (signalStage == 1) {
                System.out.println("is pont double" + ((robot.movingAverageFilter.getAverageX() * 39.37)));
                System.out.println("is pont" + ((robot.movingAverageFilter.getAverageX() * 39.37) > (16.5 + 17.5 / 2)));
                if ((robot.movingAverageFilter.getAverageX() * 39.37) > (1 + 17.5 / 2)) {
                    System.out.println(" AT point Stage1");
                    clawStartTimeMillis = robot.hwCollection.clock.getRunningTimeMillis();
                    signalStage++;
                }
            } else if (signalStage == 2) {
                robot.clawModule.setClose();
                if (robot.clawModule.getClawState().equals("Close") && clawStartTimeMillis + 1000 < robot.hwCollection.clock.getRunningTimeMillis()) {
                    System.out.println("1 sec stage 2");
                    signalStage++;
                }
            } else if (signalStage == 3) {
                robot.susanModule.goToCustomDeg(-180);
                robot.slidesModule.setTargetPositionPreset(SlidesModule.SlidePositionPreset.JUNC_2);
                if (robot.susanModule.isMoving() && robot.slidesModule.getCurPosition().getValue(Distance.Unit.INCHES) > 10) {
                    robot.clawModule.setOpen();
                    signalStage++;
                }
            } else {
                dumpSignal = false;
                signalStage = 0;
            }
        } else if (pickUpPreload) {
            System.out.println("Pick up");
            if (preloadStage == 1) {
                System.out.println("Stage1");
                robot.slidesModule.setTargetPosition(new Distance(14, Distance.Unit.INCHES));
                if (robot.slidesModule.getCurPosition().getValue(Distance.Unit.INCHES) > 12) {
                    preloadStage++;
                }
            } else if (preloadStage == 2) {
                System.out.println("Stage2");
                System.out.println("turret encoder: " + robot.hwCollection.susanMotor1.getEncoder().getEncoderPosition());
                robot.susanModule.goToPreloadedCone();
                robot.clawModule.setClawCoderOpen();
                if (robot.hwCollection.susanMotor1.getEncoder().getEncoderPosition() < -1988 && robot.susanModule.isMoving()) {
                    preloadStage++;
                }
            } else if (preloadStage == 3) {
                System.out.println("Stage3");
                robot.slidesModule.setTargetPositionPreset(SlidesModule.SlidePositionPreset.ABOVE_DRIVE);
                if (robot.slidesModule.getCurPosition().getValue(Distance.Unit.INCHES) < SlidesModule.SlidePositionPreset.ABOVE_DRIVE.HEIGHT_INCHES) {
                    preloadStage++;
                }
            } else if (preloadStage == 4) {
                System.out.println("Stage4");
                robot.clawModule.setClose();
                if (holdingCone) {
                    preloadStage++;
                }
            } else if (preloadStage == 5) {
                System.out.println("Stage5");
                robot.slidesModule.setTargetPosition(new Distance(35, Distance.Unit.INCHES));
                robot.susanModule.goToCustomDeg(90);
                if (robot.susanModule.isMoving() && robot.susanModule.getCurPosDeg() > 87 && robot.pidPositionEstimation.isNotMoving()) {
                    preloadStage++;
                }
            } else if (preloadStage == 6) {
                System.out.println("Stage6");
                robot.clawModule.setOpen();
                if (openCone) {
                    preloadStage++;
                }
            } else {
                System.out.println("Final Stage");
                pickUpPreload = false;
                preloadStage = 0;
            }
        } else if (dropCone){
            if (dropStage == 1){
                robot.clawModule.setOpen();
                if (openCone) {
                    dropStage++;
                }
            } else if (dropStage == 2) {
                System.out.println("Stage5");
                robot.susanModule.goToFront();
                robot.slidesModule.setTargetPositionPreset(SlidesModule.SlidePositionPreset.GROUND);
                if (robot.susanModule.isMoving() && robot.susanModule.getCurPosDeg() < 3) {
                    dropStage++;
                }
            } else {
                dropCone = false;
                dropStage = 0;
            }
        } else if (park){
            robot.pidPositionEstimation.stop();
            parkMillis = robot.hwCollection.clock.getRunningTimeMillis();
            if (variant == 1){
                robot.hwCollection.driveMotorFL.setPower(0.4);
                robot.hwCollection.driveMotorFR.setPower(0.4);
                robot.hwCollection.driveMotorBL.setPower(0.4);
                robot.hwCollection.driveMotorBR.setPower(0.4);
            } else if (variant == 2) {
                robot.hwCollection.driveMotorFL.setPower(0.18);
                robot.hwCollection.driveMotorFR.setPower(0.18);
                robot.hwCollection.driveMotorBL.setPower(0.18);
                robot.hwCollection.driveMotorBR.setPower(0.18);
            } else {
                robot.hwCollection.driveMotorFL.setPower(-0.18);
                robot.hwCollection.driveMotorFR.setPower(-0.18);
                robot.hwCollection.driveMotorBL.setPower(-0.18);
                robot.hwCollection.driveMotorBR.setPower(-0.18);
            }
            if (parkMillis + 4500 < robot.hwCollection.clock.getRunningTimeMillis()){
                robot.hwCollection.driveMotorFL.setPower(0);
                robot.hwCollection.driveMotorFR.setPower(0);
                robot.hwCollection.driveMotorBL.setPower(0);
                robot.hwCollection.driveMotorBR.setPower(0);
                park = false;
            }
        } else if (coneStackPickup){
            if (consPickStage == 1){
                robot.susanModule.goToFront();
                if (robot.susanModule.isMoving() && robot.susanModule.getCurPosDeg() < 45) {
                    consPickStage++;
                }
            } else if (consPickStage == 2){
                robot.slidesModule.setTargetPosition(new Distance(liftHeight, Distance.Unit.INCHES));
                if (robot.susanModule.isMoving() && robot.susanModule.getCurPosDeg() < 3 && robot.pidPositionEstimation.isNotMoving()) {
                    consPickStage++;
                }
            } else if (consPickStage == 3){
                robot.clawModule.setClose();
                if (holdingCone) {
                    consPickStage++;
                }
            } else {
                robot.slidesModule.setTargetPosition(new Distance(35, Distance.Unit.INCHES));
                robot.susanModule.goToCustomDeg(90);
                coneStackPickup = false;
                consPickStage = 0;
            }
        } else if (open) {
            if (openStage == 1) {
                robot.clawModule.setOpen();
                if (openCone) {
                    openStage++;
                }
            } else {
                open = false;
                openStage = 0;
            }
        } else if (front){
            if (frontStage == 1){
                robot.susanModule.goToFront();
                if (robot.susanModule.isMoving() && robot.susanModule.getCurPosDeg() < 45) {
                    frontStage++;
                }
            } else if (frontStage == 2){
                robot.slidesModule.setTargetPositionPreset(SlidesModule.SlidePositionPreset.GROUND);
                if (robot.susanModule.isMoving() && robot.susanModule.getCurPosDeg() < 3 && robot.pidPositionEstimation.isNotMoving()) {
                    frontStage++;
                }
            } else {
                front = false;
                frontStage = 0;
            }
        }
    }

    public synchronized void pickUpPreload() {
        System.out.println("preload HIIIIIIi");
        pickUpPreload = true;
        preloadStage = 1;
    }
    public synchronized void coneStackPickup(double height) {
        System.out.println("preload Byeee");
        coneStackPickup = true;
        this.liftHeight = height;
        consPickStage = 1;
    }

    public synchronized void open() {
        open = true;
        openStage = 1;
    }

    public synchronized void front() {
        front = true;
        frontStage = 1;
    }

    public synchronized boolean isPickingUp(){
        return pickUpPreload;
    }

    public synchronized boolean isConeStackPickup(){
        return coneStackPickup;
    }

    public synchronized boolean isOpen(){
        return open;
    }

    public synchronized void dumpSignalCone() {
        System.out.println("dump hoiiiii");
        dumpSignal = true;
        signalStage = 1;
    }

    public synchronized void dropCone() {
        dropCone = true;
        dropStage = 1;
    }

    public synchronized boolean pickUp(){
        return pickUpPreload;
    }
    public synchronized boolean drop(){
        return dropCone;
    }

    public synchronized void park(int variant){
        this.variant = variant;
        park = true;
    }

    /*
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

//    public static void runAction(int actionID, BaseRobot baseRobot) {
//        switch (actionID) {
//            case 0:
//                baseRobot.pathFollowModule.isBusy = false;
//                baseRobot.taskModule.addTask(new TaskModule.Task() {
//                    @Override
//                    public boolean loop(int runningTimeMillis, BaseRobot baseRobot) {
//                        baseRobot.stopDriveMotors();
//                        if (runningTimeMillis < 1500) {
//                            baseRobot.slidesModule.setTargetPositionPreset(SlidesModule.SlidePositionPreset.JUNC_4);
//                            return false;
//                        } else if (runningTimeMillis < 2500) {
//                            baseRobot.susanModule.goToCustomDeg(90);
//                            return false;
//                        } else if (runningTimeMillis < 3500) {
//                            baseRobot.clawModule.setOpen();
//                            return false;
//                        } else if (runningTimeMillis < 4000) {
//                            baseRobot.susanModule.goToFront();
//                            return false;
//                        } else if (runningTimeMillis < 4500) {
//                            baseRobot.slidesModule.setTargetPositionPreset(SlidesModule.SlidePositionPreset.JUNC_2);
//                            return false;
//                        } else {
//                            baseRobot.pathFollowModule.isBusy = true;
//                            return true;
//                        }
//                    }
//                });
//                break;
//            case 1:
//                baseRobot.pathFollowModule.isBusy = false;
//                baseRobot.taskModule.addTask(new TaskModule.Task() {
//                    @Override
//                    public boolean loop(int runningTimeMillis, BaseRobot baseRobot) {
//                        baseRobot.stopDriveMotors();
//                        if (runningTimeMillis < 1500) {
//                            baseRobot.slidesModule.setTargetPosition(new Distance(4.5, Distance.Unit.INCHES));
//                            return false;
//                        } else if (runningTimeMillis < 2500) {
//                            baseRobot.clawModule.setClose();
//                            return false;
//                        } else if (runningTimeMillis < 3500) {
//                            baseRobot.slidesModule.setTargetPositionPreset(SlidesModule.SlidePositionPreset.JUNC_3);
//                            return false;
//                        } else {
//                            baseRobot.pathFollowModule.isBusy = true;
//                            return true;
//                        }
//                    }
//                });
//                break;
//            case 2:
//                System.out.println("Brotato");
//                baseRobot.pathFollowModule.isBusy = false;
//                baseRobot.taskModule.addTask(new TaskModule.Task() {
//                    @Override
//                    public boolean loop(int runningTimeMillis, BaseRobot baseRobot) {
//                        baseRobot.stopDriveMotors();
//                        if (runningTimeMillis < 1500) {
//                            baseRobot.clawModule.setClose();
//                            return false;
//                        } else if (runningTimeMillis < 2000) {
//                            baseRobot.slidesModule.setTargetPositionPreset(SlidesModule.SlidePositionPreset.JUNC_1);
//                            return false;
//                        } else {
//                            baseRobot.pathFollowModule.isBusy = true;
//                            return true;
//                        }
//                    }
//                });
//                System.out.println("Brotato2");
//                break;
////            case 1:
////                baseRobot.intakeModule.setTargetPowers(INTAKE_POWER, INTAKE_POWER);
////                break;
////            case 2:
////                baseRobot.intakeModule.setTargetPowers(OUTTAKE_POWER, OUTTAKE_POWER);
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
//        }
}