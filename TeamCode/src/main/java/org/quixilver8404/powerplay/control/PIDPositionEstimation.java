package org.quixilver8404.powerplay.control;

import org.quixilver8404.powerplay.util.Vector3;

public class PIDPositionEstimation {
    BaseRobot robot;
    Vector3 point;

    double robotx     = 0;
    double roboty     = 0;
    double robotTheta = 0;

    double targetx;
    double targety;
    double targettheta;

    PIDController pidx       = new PIDController(1.2,0.00001,2,-300);
    PIDController pidy       = new PIDController(1.2,0.00001,2,-300);
    PIDController pidtheta   = new PIDController(1.2,0.00005,2,-300);





    PIDController pidhybridx = new PIDController(1.2,0.00001,2,-300);
    PIDController pidhybridy = new PIDController(1.2,0.00001,2,-300);

    boolean moveX;
    boolean moveY;
    boolean moveTheta;
    boolean moveHybridx;
    boolean moveHybridy;

    public PIDPositionEstimation(BaseRobot robot, Vector3 point) {
        this.robot = robot;
        this.point = point;
    }

    public synchronized void setPoint(Vector3 point) {
        this.point = point;
    }

    public synchronized void update(){
        targetx = 0;
        targety = 0;
        targettheta = 0;

        if (moveX) {
            double offsetX = robot.movingAverageFilter.getAverageX() - point.x();

            robotx = pidx.loop(offsetX, robot.hwCollection.clock.getRunningTimeMillis());
            if (Math.abs(robotx) < 0.2){
                robotx = Math.signum(robotx)*0.2;
            }
            targetx = robotx;
//            System.out.println("robotX: " + robotx);
//            System.out.println("Offsetx: " + offsetX);
//            System.out.println("Point: " + point);
//            System.out.println("Robot'x: " + robot.movingAverageFilter.getAverageX());
            if (Math.abs(offsetX) < 0.03) {
                targetx = 0;
                pidx.reset();
                moveX = false;
                robot.driveModule.setIntrinsicTargetPower(0,0);
            }
        }
        if (moveTheta){
            double offsetTheta = robot.movingAverageFilter.getAverageTheta() - point.theta();

            robotTheta = pidtheta.loop(offsetTheta, robot.hwCollection.clock.getRunningTimeMillis());
            if (Math.abs(robotTheta) < 0.12) {
                robotTheta = Math.signum(robotTheta) * 0.12;
            }
            targettheta = robotTheta;
            if (Math.abs(offsetTheta) < 0.015) {
                targettheta = 0;
                pidtheta.reset();
                moveTheta = false;
                robot.driveModule.setTargetRotatePower(0);
            }
        }
        if (moveY) {
            double offsetY = robot.movingAverageFilter.getAverageY() - point.y();
            roboty = pidy.loop(offsetY, robot.hwCollection.clock.getRunningTimeMillis());
            if (Math.abs(roboty) < 0.2){
                roboty = Math.signum(roboty)*0.2;
            }
            targety = roboty;
            System.out.println("robotY: " + roboty);
            System.out.println("Offsety: " + offsetY);

            if (Math.abs(offsetY) < 0.03) {
                targety = 0;
                pidy.reset();
                moveY = false;
                robot.driveModule.setIntrinsicTargetPower(0, 0);
            }
        }
        if (moveHybridx) {
            double offsetX = point.y() - robot.movingAverageFilter.getAverageY();

            robotx = pidhybridx.loop(offsetX, robot.hwCollection.clock.getRunningTimeMillis());
            if (Math.abs(robotx) < 0.2){
                robotx = Math.signum(robotx)*0.2;
            }
            targetx = robotx;
            if (Math.abs(offsetX) < 0.03) {
                targetx = 0;
                pidhybridx.reset();
                moveHybridx = false;
                robot.driveModule.setIntrinsicTargetPower(0, 0);
            }
        }
        if (moveHybridy) {
            double offsetY = point.x() - robot.movingAverageFilter.getAverageX();

            roboty = pidhybridy.loop(offsetY, robot.hwCollection.clock.getRunningTimeMillis());
            if (Math.abs(roboty) < 0.2){
                roboty = Math.signum(roboty)*0.2;
            }
            targety = roboty;
            if (Math.abs(offsetY) < 0.03) {
                targety = 0;
                pidhybridy.reset();
                moveHybridy = false;
                robot.driveModule.setIntrinsicTargetPower(0, 0);
            }
        }
        if (moveX || moveY || moveTheta || moveHybridx || moveHybridy) {
            robot.driveModule.setTargetRotatePower(targettheta);
            robot.driveModule.setIntrinsicTargetPower(targetx, targety);
        }
    }

    public synchronized void goHybridX() {
        moveHybridx = true;
    }

    public synchronized void goHybridY() {
        moveHybridy = true;
    }

    public synchronized void goX(){
        moveX = true;
    }

    public synchronized void goY(){
        moveY = true;
    }

    public synchronized void goTheta(){
        moveTheta = true;
    }

    public synchronized void go() {
        moveTheta = true;
        moveX = true;
        moveY = true;
    }

    public synchronized boolean isNotMoving(){
        return !(moveTheta || moveX || moveY || moveHybridx || moveHybridy);
    }

    public synchronized Vector3 getPoint(){
        return point;
    }

    public synchronized void stop(){
        moveY = false;
        moveX = false;
        moveTheta = false;
        moveHybridx = false;
        moveHybridy = false;
    }

}
