package org.quixilver8404.powerplay.opmode.auton;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.quixilver8404.powerplay.control.AutonRobot;
import org.quixilver8404.powerplay.util.Vector3;

@Autonomous(group = "Good")
public class RepeatedMotion extends LinearOpMode {
    int variant = 0;
    double yPos;
    AutonRobot robot;
    int counter = 0;
    int startTimer;

    @Override
    public void runOpMode() throws InterruptedException {

        telemetry.addData("status", "initializing hardware...");
        telemetry.update();

        robot = new AutonRobot(new Vector3(17.5/2 * 0.0254,(141-15) *0.0254,-90),this);
        robot.startHardwareLoop();

        robot.headingLockModule.enablePID(robot);

//        robot.hwCollection.camera.setPipeline(robot.cvTasksModule);

        while (opModeInInit()) {
            variant = robot.cvTasksModule.getVariant();
            telemetry.addData("Auton Var", variant);

            telemetry.update();
        }
        robot.poseModule.setStartPos(new Vector3(0 * 0.0254,(100) * 0.0254,0));

        if (isStopRequested()) {
            robot.stopHardwareLoop();
            return;
        }
        while (counter <= 50 && opModeIsActive()) {
            robot.pidPositionEstimation.setPoint(new Vector3(20 * 0.0254, 100 * 0.0254, 0));
            robot.pidPositionEstimation.goX();
            robot.pidPositionEstimation.goY();
            robot.pidPositionEstimation.goTheta();
            while (!robot.pidPositionEstimation.isNotMoving() && opModeIsActive()) {
                telemetry();
            }
            startTimer = robot.hwCollection.clock.getRunningTimeMillis();
            while (startTimer + 1000 > robot.hwCollection.clock.getRunningTimeMillis() && opModeIsActive()) {
                telemetry();
            }
            robot.pidPositionEstimation.setPoint(new Vector3(0 * 0.0254,(100) * 0.0254,0));
            robot.pidPositionEstimation.goX();
            robot.pidPositionEstimation.goY();
            robot.pidPositionEstimation.goTheta();
            while (!robot.pidPositionEstimation.isNotMoving() && opModeIsActive()) {
                telemetry();
            }
            startTimer = robot.hwCollection.clock.getRunningTimeMillis();
            while (startTimer + 1000 > robot.hwCollection.clock.getRunningTimeMillis() && opModeIsActive()) {
                telemetry();
            }
            counter++;
        }
        while (opModeIsActive()) {
            telemetry();
        }
    }

    public void telemetry() {
        telemetry.addData("counter", counter);
        telemetry.addData("pos", robot.poseModule.getPos().toString());
        telemetry.addData("curPosDeg", robot.susanModule.getCurPosDeg());
        telemetry.addData("susan power", robot.hwCollection.susanMotor1.getPower());
        telemetry.addData("loop frequency", "%dHz", robot.diagnosticModule.getLoopFrequencyHz());
        telemetry.addData("left encoder", robot.hwCollection.driveEncoderLeft.getEncoderPosition());
        telemetry.addData("right encoder", robot.hwCollection.driveEncoderRight.getEncoderPosition());
        telemetry.addData("center encoder", robot.hwCollection.driveEncoderCenter.getEncoderPosition());
        telemetry.addData("x", robot.movingAverageFilter.getAverageX() * 39.37);
        telemetry.addData("y", robot.movingAverageFilter.getAverageY() * 39.37);
        telemetry.addData("heading", robot.movingAverageFilter.getAverageTheta());
//        telemetry.addData("ultrasonic1 dist", robot.hwCollection.ultraSonic1.getDistance(DistanceUnit.INCH));
//        telemetry.addData("ultrasonic2 dist", robot.hwCollection.ultraSonic2.getDistance(DistanceUnit.INCH));
//        telemetry.addData("ultrasonic3 dist", robot.hwCollection.ultraSonic3.getDistance(DistanceUnit.INCH));
//        telemetry.addData("ultrasonic pos", Arrays.toString(MSonicModule.tripleThreat(
//                robot.movingAverageFilter.getAverageX() * 39.37,
//                robot.movingAverageFilter.getAverageY() * 39.37,
//                robot.movingAverageFilter.getAverageTheta(),
//                robot.hwCollection.ultraSonic1.getDistance(DistanceUnit.INCH),
//                robot.hwCollection.ultraSonic2.getDistance(DistanceUnit.INCH),
//                robot.hwCollection.ultraSonic3.getDistance(DistanceUnit.INCH))));
        telemetry.addData("power settings", robot.hwCollection.driveMotorFL.getPower() + ", "
                + robot.hwCollection.driveMotorFR.getPower() + ", "
                + robot.hwCollection.driveMotorBL.getPower() + ", "
                + robot.hwCollection.driveMotorBR.getPower());
        telemetry.update();
        System.out.println(robot.hwCollection.driveMotorFL.getPower() + ", "
                + robot.hwCollection.driveMotorFR.getPower() + ", "
                + robot.hwCollection.driveMotorBL.getPower() + ", "
                + robot.hwCollection.driveMotorBR.getPower());
    }
}
