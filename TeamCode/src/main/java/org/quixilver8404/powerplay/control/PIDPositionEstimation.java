package org.quixilver8404.powerplay.control;

import org.quixilver8404.powerplay.util.Vector3;

public class PIDPositionEstimation {
    BaseRobot robot;
    Vector3 point;

    double robotx = 0;
    double roboty = 0;
    double robotTheta = 0;

    double targetx;
    double targety;
    double targettheta;

    PIDController pidx = new PIDController(0.8,0,1,-300);
    PIDController pidy = new PIDController(0.8,0,1,-300);
    PIDController pidtheta = new PIDController(1.2, 0, 1,-300);
    PIDController pidhybrid = new PIDController(0.6,0,1,-300);

    boolean moveX;
    boolean moveY;
    boolean moveTheta;
    boolean goHybrid;

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
            if (Math.abs(robotx) < 0.15){
                robotx = Math.signum(robotx)*0.15;
            }
            targetx = robotx;
            System.out.println("robotX: " + robotx);
            System.out.println("Offsetx: " + offsetX);
            System.out.println("Point: " + point);
            System.out.println("Robot'x: " + robot.movingAverageFilter.getAverageX());
            if (Math.abs(offsetX) < 0.03) {
                targetx = 0;
                pidx.reset();
                moveX = false;
            }
        }
        if (moveTheta){
            double offsetTheta = -point.theta() - robot.movingAverageFilter.getAverageTheta();

            robotTheta = pidtheta.loop(offsetTheta, robot.hwCollection.clock.getRunningTimeMillis());
            targettheta = robotTheta;
            if (Math.abs(robotTheta) < 0.15){
                robotTheta = Math.signum(robotTheta) * 0.15;
            }
            if (Math.abs(offsetTheta) < 0.05) {
                targettheta = 0;
                pidtheta.reset();
                moveTheta = false;
            }
        }
        if (moveY) {
            double offsetY = robot.movingAverageFilter.getAverageY() - point.y();
            roboty = pidy.loop(offsetY, robot.hwCollection.clock.getRunningTimeMillis());
            targety = roboty;
            System.out.println("robotY: " + roboty);
            System.out.println("Offsety: " + offsetY);

            if (Math.abs(offsetY) < 0.03) {
                targety = 0;
                pidy.reset();
                moveY = false;
            }
        }
        if (goHybrid) {
            double offsetX = point.y() - robot.movingAverageFilter.getAverageY();

            robotx = pidhybrid.loop(offsetX, robot.hwCollection.clock.getRunningTimeMillis());
            if (Math.abs(robotx) < 0.15) {
                robotx = Math.signum(robotx) * 0.1;
            }
            targetx = robotx;
            if (Math.abs(offsetX) < 0.03) {
                targetx = 0;
                pidhybrid.reset();
                moveX = false;
            }
        }
        if (moveX || moveY || goHybrid || moveTheta) {
            robot.driveModule.setTargetRotatePower(targettheta);
            robot.driveModule.setExtrinsicTargetPower(targetx, targety);
        }
    }

    public synchronized void goHybrid() {
        goHybrid = true;
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

    public synchronized boolean getMove(){
        return !(moveTheta || moveX || moveY);
    }

    public synchronized Vector3 getPoint(){
        return point;
    }

    public synchronized void stop(){
        moveY = false;
        moveX = false;
        moveTheta = false;
    }

}
