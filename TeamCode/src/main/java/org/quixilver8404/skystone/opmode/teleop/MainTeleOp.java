package org.quixilver8404.skystone.opmode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.quixilver8404.skystone.control.SlidesModule;
import org.quixilver8404.skystone.control.TeleOpRobot;
import org.quixilver8404.skystone.util.measurement.Angle;
import org.quixilver8404.skystone.util.measurement.Distance;
import org.quixilver8404.skystone.util.measurement.Pose2D;

import java.util.Arrays;

@TeleOp
public class MainTeleOp extends LinearOpMode {

    public static final double BASE_DRIVE_POWER = 1;
    public static final double BASE_ROTATE_POWER = 1;

    public static final double SLOW_DRIVE_POWER = 0.2;
    public static final double SLOW_ROTATE_POWER = 0.2;

    public static final double FAST_DRIVE_POWER = 1.0;
    public static final double FAST_ROTATE_POWER = 1.0;

    // (final power) = (original power)^(exponent)
    public static final double DRIVE_POWER_EXPONENT = 1.0;
    public static final double ROTATE_POWER_EXPONENT = 1.0;

    // velocity to use for dpad drive control
    public static final double DPAD_DRIVE_POWER_FORWARD = 0.3;
    public static final double DPAD_DRIVE_POWER_STRAFE = 0.5;
    public static final double DPAD_DRIVE_POWER_ROTATE = 0.3;

    public static boolean frontFlipped = true;
    public static boolean flipFrontReleased = true;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("status", "initializing hardware...");
        telemetry.update();

        final TeleOpRobot robot = new TeleOpRobot(new Pose2D(
                new Distance(12.5, Distance.Unit.INCHES),
                new Distance(12.5, Distance.Unit.INCHES),
                new Angle(0, Angle.Unit.DEGREES)
        ), this);

        // starts the robot hardware update loop
        robot.headingLockModule.disablePID();
        robot.startHardwareLoop();

        telemetry.addData("status", "ready!");
        telemetry.update();

        waitForStart();

        telemetry.addData("status", "running");
        telemetry.update();

//        robot.headingLockModule.enablePID(robot);

        while (opModeIsActive()) {
            //==========================DRIVER ONE==================================================================

            //add telemetry stuffs here
            double x = gamepad1.left_stick_x;
            // For some reason, the controllers or sdk or whatever flips up with down ¯\_(ツ)_/¯
            double y = -gamepad1.left_stick_y;

            telemetry.addData("x", x);
            telemetry.addData("y", y);

            double powerModifier = gamepad1.left_trigger;
            double rotateModifier = -gamepad1.right_trigger;

            // calculates the magnitude relative to the maximum value of the joystick
            // at the current angle
            // this is equivalent to the larger magnitude of x and y
            double magnitude = Math.max(Math.abs(x), Math.abs(y));

            // set the magnitude to scale based on the left trigger
            // scales between the maxLowMagnitude to 1 based on the left trigger
            if (gamepad1.left_bumper) {
                magnitude *= SLOW_DRIVE_POWER;
                magnitude *= (1 + (((1 / SLOW_DRIVE_POWER) - 1) * powerModifier));
            } else {
                magnitude *= BASE_DRIVE_POWER;
                magnitude *= (1 + powerModifier);
            }

            double angle = Math.atan2(y, x);

            if (frontFlipped) {
                angle = Angle.toStandard(angle + Math.PI, Angle.Unit.RADIANS);
            }

            angle = Angle.toStandard(angle + Math.PI / 2, Angle.Unit.RADIANS);

            // moving the joystick right should give a negative rotation (clockwise)
            double rotatePower = -gamepad1.right_stick_x;
            rotatePower = Math.signum(rotatePower) * Math.sqrt(Math.abs(rotatePower));

            rotatePower = Math.pow(rotatePower, ROTATE_POWER_EXPONENT);

            // set the rotation power to scale based on the right trigger
            // scales between the maxLowMagnitude to 1 based on the left trigger

            if (gamepad1.right_bumper) {
                rotatePower *= SLOW_ROTATE_POWER;
                rotatePower *= (1 + (((1 / SLOW_ROTATE_POWER) - 1) * rotateModifier));
            } else {
                rotatePower *= BASE_ROTATE_POWER;
                rotatePower *= (1 + rotateModifier);
            }

            robot.driveModule.setIntrinsicTargetPower(magnitude, new Angle(angle, Angle.Unit.RADIANS));
            robot.driveModule.setTargetRotatePower(rotatePower);

            telemetry.addData("Intrinsic target vector", "" + magnitude + ", " + angle);
            telemetry.addData("Rotate target power", "" + rotatePower);
            telemetry.addData("FR", robot.hwCollection.driveMotorFR.getPower());
            telemetry.addData("FL", robot.hwCollection.driveMotorFL.getPower());
            telemetry.addData("BL", robot.hwCollection.driveMotorBL.getPower());
            telemetry.addData("BR", robot.hwCollection.driveMotorBR.getPower());

            // handle flipping front
            if (gamepad1.y) {
                if (flipFrontReleased) {
                    frontFlipped = !frontFlipped;
                    flipFrontReleased = false;
                }
            } else {
                flipFrontReleased = true;
            }

            //==========================DRIVER TWO==================================================================
            if (gamepad2.left_bumper) {
                robot.clawModule.setClose();
            }
            if (gamepad2.right_bumper) {
                robot.clawModule.setOpen();
            }
            if (gamepad2.y) {
                robot.slidesModule.setTargetPositionPreset(SlidesModule.SlidePositionPreset.JUNC_4);
                robot.susanModule.goToCustomDeg(90);
            }
            if (gamepad2.x) {
                robot.slidesModule.setTargetPositionPreset(SlidesModule.SlidePositionPreset.JUNC_4);
                robot.susanModule.goToCustomDeg(-90);
            }
            if (gamepad2.b) {
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

            // ==================== TELEMETRY ====================
            // TODO remove after testing
            telemetry.addData("lift height", "%f in", robot.slidesModule.getCurPosition().getValue(Distance.Unit.INCHES));
            telemetry.addData("susan position", robot.hwCollection.susanMotor1.getEncoder().getEncoderPosition());
            telemetry.addData("loop frequency", "%dHz", robot.diagnosticModule.getLoopFrequencyHz());
            telemetry.addData("left encoder", robot.hwCollection.driveEncoderLeft.getEncoderPosition());
            telemetry.addData("right encoder", robot.hwCollection.driveEncoderRight.getEncoderPosition());
            telemetry.addData("center encoder", robot.hwCollection.driveEncoderCenter.getEncoderPosition());
            telemetry.addData("x", "%f in", robot.navModule.getPose().x.getValue(Distance.Unit.INCHES));
            telemetry.addData("y", "%f in", robot.navModule.getPose().y.getValue(Distance.Unit.INCHES));
            telemetry.addData("heading", "%f deg", robot.navModule.getHeading().getStandard(Angle.Unit.DEGREES));
//            telemetry.addData("ultrasonic dist", robot.hwCollection.ultraSonic1.getDistance(DistanceUnit.INCH));
//            telemetry.addData("ultrasonic pos", Arrays.toString(robot.mSonicModule.calcPos(
//                    robot.navModule.getPose().x.getValue(Distance.Unit.INCHES),
//                    robot.navModule.getPose().y.getValue(Distance.Unit.INCHES),
//                    robot.navModule.getHeading().getStandard(Angle.Unit.RADIANS),
//                    robot.hwCollection.ultraSonic1.getDistance(DistanceUnit.INCH),
//                    (15.0 / 2 - 2.0), (17.5 / 2 - 2.625), -Math.PI / 2)));
            telemetry.update();
        }
        robot.stopHardwareLoop();
    }
}
