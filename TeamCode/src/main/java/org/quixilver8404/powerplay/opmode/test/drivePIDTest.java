package org.quixilver8404.powerplay.opmode.test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.quixilver8404.powerplay.control.AutonRobot;
import org.quixilver8404.powerplay.util.Vector3;
import org.quixilver8404.powerplay.util.measurement.Angle;
import org.quixilver8404.powerplay.util.measurement.Distance;

@Autonomous(name = "drive PID test", group = "Test")
public class drivePIDTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("status", "initializing hardware...");
        telemetry.update();

        AutonRobot robot = new AutonRobot(this);
        robot.headingLockModule.disablePID();
        robot.startHardwareLoop();

        while (opModeInInit()) {
            robot.pidPositionEstimation.setPoint(new Vector3(0,0, Math.PI));
            telemetry.addData("status", "ready");
            telemetry.update();
        }

        if (isStopRequested()) {
            robot.stopHardwareLoop();
            robot.stopDriveMotors();
            return;
        }



        while (opModeIsActive()) {
            telemetry.addData("left encoder", robot.hwCollection.driveEncoderLeft.getEncoderPosition());
            telemetry.addData("right encoder", robot.hwCollection.driveEncoderRight.getEncoderPosition());
            telemetry.addData("center encoder", robot.hwCollection.driveEncoderCenter.getEncoderPosition());
            telemetry.addData("Position", robot.poseModule.getPos());
            telemetry.update();
            robot.pidPositionEstimation.goTheta();
        }
        robot.stopHardwareLoop();
        robot.stopDriveMotors();
    }
}
