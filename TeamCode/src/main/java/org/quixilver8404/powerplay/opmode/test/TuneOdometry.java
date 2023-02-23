package org.quixilver8404.powerplay.opmode.test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.quixilver8404.powerplay.control.SlidesModule;
import org.quixilver8404.powerplay.control.TeleOpRobot;
import org.quixilver8404.powerplay.util.ImageOutput;
import org.quixilver8404.powerplay.util.Vector3;
import org.quixilver8404.powerplay.util.measurement.Angle;
import org.quixilver8404.powerplay.util.measurement.Distance;
import org.quixilver8404.powerplay.util.measurement.Pose2D;

import java.util.Arrays;
import java.util.Random;

@TeleOp(name = "Tune Odometry", group = "Test")
public class TuneOdometry extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("status", "initializing hardware...");
        telemetry.update();

        final TeleOpRobot robot = new TeleOpRobot(new Vector3(),this);
        robot.slidesModule.teleOpMode();

        // starts the robot hardware update loop
        robot.headingLockModule.disablePID();
        robot.startHardwareLoop();

        telemetry.addData("status", "ready!");
        telemetry.update();
        robot.clawModule.setClose();

        waitForStart();

        telemetry.addData("status", "running");
        telemetry.update();

        while (opModeIsActive()) {
            if (gamepad1.dpad_up) {
                robot.driveModule.setIntrinsicTargetPower(1, 0);
            } else if (gamepad1.dpad_down) {
                robot.driveModule.setIntrinsicTargetPower(-1,0);
            } else if (gamepad1.dpad_left) {
                robot.driveModule.setIntrinsicTargetPower(0,1);
            } else if (gamepad1.dpad_right) {
                robot.driveModule.setIntrinsicTargetPower(0,-1);
            } else {
                robot.driveModule.setIntrinsicTargetPower(0,0);
            }
            telemetry.addData("left", robot.hwCollection.driveEncoderLeft.getEncoderPosition());
            telemetry.addData("right", robot.hwCollection.driveEncoderRight.getEncoderPosition());
            telemetry.addData("center", robot.hwCollection.driveEncoderCenter.getEncoderPosition());
            telemetry.update();
        }
        robot.stopHardwareLoop();
    }
}
