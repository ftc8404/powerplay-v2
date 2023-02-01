package org.quixilver8404.powerplay.control;

import org.quixilver8404.powerplay.util.Vector3;

public class PIDPositionEstimation {
    BaseRobot robot;
    Vector3 point;

    double robotx;
    double roboty;
    double robotz;

    int timeoutMillis = 0;

    public PIDPositionEstimation(BaseRobot robot, Vector3 point) {
        this.robot = robot;
        this.point = point;
    }

    public synchronized void setPoint(Vector3 point) {
        this.point = point;
    }

    public synchronized void goToPoint() {

        PIDController pidx = new PIDController(4,0.3,2,300);
        PIDController pidy = new PIDController(4,0.3,2,300);
        PIDController pidtheta = new PIDController(4, 0.03, 2,300);

        int prevTime = 0;

        do {
            System.out.println("While");
            if (robot.hwCollection.clock.getRunningTimeMillis() != prevTime) {
                System.out.println("left encoder"+ robot.hwCollection.driveEncoderLeft.getEncoderPosition());
                System.out.println("right encoder"+ robot.hwCollection.driveEncoderRight.getEncoderPosition());
                System.out.println("center encoder"+ robot.hwCollection.driveEncoderCenter.getEncoderPosition());
                System.out.println("Position"+ robot.poseModule.getPos());
                robotx = pidx.loop(robot.poseModule.getPosX() - point.x(), robot.hwCollection.clock.getRunningTimeMillis());
                roboty = pidy.loop(robot.poseModule.getPosY() - point.y(), robot.hwCollection.clock.getRunningTimeMillis());
                robotz = pidtheta.loop(robot.poseModule.getPosZ() - point.theta(), robot.hwCollection.clock.getRunningTimeMillis());

                System.out.println("RobotX: " + robotx);
                System.out.println("RobotY: " + roboty);
                System.out.println("RobotZ: " + robotz);


                robot.driveModule.setExtrinsicTargetPower(robotx, roboty);
                robot.driveModule.setTargetRotatePower(robotz);
                prevTime = robot.hwCollection.clock.getRunningTimeMillis();
            }
        } while (!(robotx < 0.01 && roboty < 0.01 && robotz < 0.01));

//        robot.taskModule.addTask(new TaskModule.Task() {
//            @Override
//            public boolean loop(int runningTimeMillis, BaseRobot baseRobot) {
//                robotx = pidx.loop(robot.poseModule.getPosX() - point.x(), robot.hwCollection.clock.getRunningTimeMillis());
//                roboty = pidy.loop(robot.poseModule.getPosY() - point.y(), robot.hwCollection.clock.getRunningTimeMillis());
//                robotz = pidtheta.loop(robot.poseModule.getPosZ() - point.theta(), robot.hwCollection.clock.getRunningTimeMillis());
////                System.out.println("RobotX: " + robotx);
////                System.out.println("RobotY: " + roboty);
////                System.out.println("RobotZ: " + robotz);
//                robot.driveModule.setExtrinsicTargetPower(robotx, roboty);
//                robot.driveModule.setTargetRotatePower(robotz);
//
//                return robotx < 0.01 && roboty < 0.01 && robotz < 0.01;
////                robot.hwCollection.driveMotorBR.setPower();
//            }
//        });

        pidx.reset();
        pidy.reset();
        pidtheta.reset();
    }

}
