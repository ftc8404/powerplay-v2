package org.quixilver8404.powerplay.opmode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.quixilver8404.powerplay.control.SlidesModule;
import org.quixilver8404.powerplay.control.TeleOpRobot;
import org.quixilver8404.powerplay.util.Vector3;
import org.quixilver8404.powerplay.util.measurement.Angle;
import org.quixilver8404.powerplay.util.measurement.Distance;
import org.quixilver8404.powerplay.util.measurement.Pose2D;

@TeleOp
public class TestServo extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        final TeleOpRobot robot = new TeleOpRobot(new Vector3(), this);

        // starts the robot hardware update loop
        robot.headingLockModule.disablePID();
        robot.startHardwareLoop();

        waitForStart();

        while (opModeIsActive()) {
            if (gamepad2.x) {
                robot.clawModule.setClose();
            }
            if (gamepad2.y) {
                robot.clawModule.setOpen();
            }
            if (gamepad2.left_trigger > 0) {
                robot.susanModule.goToFront();
            } else {
                robot.susanModule.setManualPower(gamepad2.right_stick_x);
            }
            if (gamepad2.dpad_down) {
                robot.slidesModule.setTargetPositionPreset(SlidesModule.SlidePositionPreset.GROUND);
            } else if (gamepad2.dpad_left) {
                robot.slidesModule.setTargetPositionPreset(SlidesModule.SlidePositionPreset.JUNC_1);
            } else if (gamepad2.dpad_right) {
                robot.slidesModule.setTargetPositionPreset(SlidesModule.SlidePositionPreset.JUNC_2);
            } else if (gamepad2.dpad_up) {
                robot.slidesModule.setTargetPositionPreset(SlidesModule.SlidePositionPreset.JUNC_3);
            } else if (gamepad2.right_trigger > 0) {
                robot.slidesModule.setTargetPositionPreset(SlidesModule.SlidePositionPreset.JUNC_4);
            } else {
                robot.slidesModule.setTargetPower(-gamepad2.left_stick_y);
            }
            telemetry.addData("Claw state", robot.clawModule.getClawState());
            telemetry.addData("slide pos", robot.slidesModule.getCurPosition().getValue(Distance.Unit.INCHES));
            telemetry.addData("slide pos", robot.hwCollection.slidesMotor1.getEncoder().getEncoderPosition());
            telemetry.addData("susan state", robot.susanModule.getSusanControlState());
            telemetry.addData("susan pos", robot.hwCollection.susanMotor1.getEncoder().getEncoderPosition());
            telemetry.update();
        }
        robot.stopHardwareLoop();
    }
}
