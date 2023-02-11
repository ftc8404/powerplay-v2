package org.quixilver8404.powerplay.control;

import org.quixilver8404.powerplay.util.Vector3;

public class PIDPositionEstimation {
    BaseRobot robot;
    Vector3 point;

    double robotx = 0;
    double roboty = 0;
    double robotTheta = 0;
    double robotHybrid = 0;

    int timeoutMillis = 0;

    PIDController pidx = new PIDController(0.8,0,1,-300);
    PIDController pidy = new PIDController(0.8,0,1,300);
    PIDController pidtheta = new PIDController(1.3, 0, 0,300);
    PIDController pidHybrid = new PIDController(0.8,0,0,-300);

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
        if (moveX) {
            double offsetX = robot.movingAverageFilter.getAverageX() - point.x();

            robotx = pidx.loop(offsetX, robot.hwCollection.clock.getRunningTimeMillis());
            if (Math.abs(robotx) < 0.15){
                robotx = Math.signum(robotx)*0.15;
            }
            robot.driveModule.setIntrinsicTargetPower(robotx, 0);
            System.out.println("robotX: " + robotx);
            System.out.println("Offsetx: " + offsetX);
            System.out.println("Point: " + point);
            System.out.println("Robot'x: " + robot.movingAverageFilter.getAverageX());
            if (Math.abs(offsetX) < 0.03) {
                robot.driveModule.setExtrinsicTargetPower(0, 0);
                pidx.reset();
                moveX = false;
            }
        } else if (moveTheta){
            double offsetTheta = -point.theta() - robot.movingAverageFilter.getAverageTheta();

            robotTheta = pidtheta.loop(offsetTheta, robot.hwCollection.clock.getRunningTimeMillis());
            robot.driveModule.setTargetRotatePower(robotTheta);
            if (Math.abs(offsetTheta) < 0.03) {
                robot.driveModule.setTargetRotatePower(0);
                pidtheta.reset();
                moveTheta = false;
            }
        } else if (moveY) {
            double offsetY = robot.movingAverageFilter.getAverageY() - point.y();
            roboty = pidy.loop(offsetY, robot.hwCollection.clock.getRunningTimeMillis());
            robot.driveModule.setIntrinsicTargetPower(0, roboty);
            System.out.println("robotY: " + roboty);
            System.out.println("Offsety: " + offsetY);

            if (Math.abs(offsetY) < 0.2) {
                robot.driveModule.setIntrinsicTargetPower(0, 0);
                pidy.reset();
                moveY = false;
            }
        } else if (goHybrid) {
            double offsetHybrid = robot.movingAverageFilter.getAverageX() + point.x();
            robotHybrid = pidHybrid.loop(offsetHybrid, robot.hwCollection.clock.getRunningTimeMillis());
            robot.driveModule.setIntrinsicTargetPower(-robotHybrid, 0);
            System.out.println("robotHybrid: " + robotHybrid);
            System.out.println("OffsetHybrid: " + offsetHybrid);

            if (Math.abs(offsetHybrid) < 0.03) {
                robot.driveModule.setIntrinsicTargetPower(0,0);
                pidHybrid.reset();
                goHybrid = false;
            }
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
