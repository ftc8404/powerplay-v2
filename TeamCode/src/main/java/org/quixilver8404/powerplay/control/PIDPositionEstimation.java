package org.quixilver8404.powerplay.control;

import org.quixilver8404.powerplay.util.Vector3;

public class PIDPositionEstimation {
    BaseRobot robot;
    Vector3 point;

    double robotx = 0;
    double roboty = 0;
    double robotTheta = 0;

    int timeoutMillis = 0;

    PIDController pidx = new PIDController(4,0.3,2,300);
    PIDController pidy = new PIDController(4,0.3,2,300);
    PIDController pidtheta = new PIDController(1.2, 0, 1,300);

    boolean moveXY;
    boolean moveTheta;

    public PIDPositionEstimation(BaseRobot robot, Vector3 point) {
        this.robot = robot;
        this.point = point;
    }

    public synchronized void setPoint(Vector3 point) {
        this.point = point;
    }

    public synchronized void update(){
        if (moveXY) {
            robotx = pidx.loop(robot.poseModule.getPosX() - point.x(), robot.hwCollection.clock.getRunningTimeMillis());
            roboty = pidy.loop(robot.poseModule.getPosY() - point.y(), robot.hwCollection.clock.getRunningTimeMillis());
            robot.driveModule.setExtrinsicTargetPower(robotx, roboty);
            if (robotx < 0.01 && roboty < 0.01) {
                robot.driveModule.setExtrinsicTargetPower(0, 0);
                pidtheta.reset();
                moveXY = false;
            }
        } else if (moveTheta){
            robotTheta = pidtheta.loop(robot.poseModule.getPosTheta() - point.theta(), robot.hwCollection.clock.getRunningTimeMillis());
            robot.driveModule.setTargetRotatePower(robotTheta);
            if (robotTheta < 0.01) {
                robot.driveModule.setTargetRotatePower(0);
                pidtheta.reset();
                moveTheta = false;
            }
        }
    }

    public synchronized void goXY(){
        moveXY = true;
    }
    public synchronized void goTheta(){
        moveTheta = true;
    }

    public synchronized boolean getMove(){
        return !(moveTheta || moveXY);
    }

}
