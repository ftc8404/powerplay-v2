package org.quixilver8404.powerplay.opmode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.quixilver8404.powerplay.control.ClawModule;
import org.quixilver8404.powerplay.control.SlidesModule;
import org.quixilver8404.powerplay.control.TeleOpRobot;
import org.quixilver8404.powerplay.util.ImageOutput;
import org.quixilver8404.powerplay.util.Vector3;
import org.quixilver8404.powerplay.util.measurement.Angle;
import org.quixilver8404.powerplay.util.measurement.Distance;
import org.quixilver8404.powerplay.util.measurement.Pose2D;

import java.util.Arrays;
import java.util.Random;

@TeleOp(name = "Main Tele-Op", group = "Main")
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

    public static double prevClawCoder;
    public static int counter = 0;

    public static boolean frontFlipped = true;
    public static boolean flipFrontReleased = true;
    static boolean firstIteration = true;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("status", "initializing hardware...");
        telemetry.update();

        final TeleOpRobot robot = new TeleOpRobot(new Vector3(0*0.0254,0*0.0254,0),this);
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
        robot.hwCollection.imu.resetYaw();
        robot.mSonicModule.setConfig(4);
        robot.clawModule.setClawCoderClose();
        prevClawCoder = robot.hwCollection.clawCoder.getEncoderPosition();

//        robot.headingLockModule.enablePID(robot);

        while (opModeIsActive()) {
            YawPitchRollAngles orientation = robot.hwCollection.imu.getRobotYawPitchRollAngles();

            // =================================================================================================================================================
            // =================================================================================================================================================
            // =================================================================================================================================================


            //add telemetry stuffs here
            double x = gamepad1.left_stick_x;
            // For some reason, the controllers or sdk or whatever flips up with down ¯\_(ツ)_/¯
            double y = -gamepad1.left_stick_y;

//            telemetry.addData("x", x);
//            telemetry.addData("y", y);

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

            double rotatePower = gamepad1.right_stick_x;
            rotatePower = Math.signum(rotatePower) * Math.sqrt(Math.abs(rotatePower));

            rotatePower = Math.pow(rotatePower, ROTATE_POWER_EXPONENT);

            if (gamepad1.right_bumper) {
                rotatePower *= SLOW_ROTATE_POWER;
                rotatePower *= (1 + (((1 / SLOW_ROTATE_POWER) - 1) * rotateModifier));
            } else {
                rotatePower *= BASE_ROTATE_POWER;
                rotatePower *= (1 + rotateModifier);
            }

            robot.driveModule.setIntrinsicTargetPower(magnitude, new Angle(angle, Angle.Unit.RADIANS));
            robot.driveModule.setTargetRotatePower(rotatePower);

//            telemetry.addData("Intrinsic target vector", "" + magnitude + ", " + angle);
//            telemetry.addData("Rotate target power", "" + rotatePower);
//            telemetry.addData("FR", robot.hwCollection.driveMotorFR.getPower());
//            telemetry.addData("FL", robot.hwCollection.driveMotorFL.getPower());
//            telemetry.addData("BL", robot.hwCollection.driveMotorBL.getPower());
//            telemetry.addData("BR", robot.hwCollection.driveMotorBR.getPower());

            // handle flipping front
            if (gamepad1.y) {
                if (flipFrontReleased) {
                    frontFlipped = !frontFlipped;
                    flipFrontReleased = false;
                }
            } else {
                flipFrontReleased = true;
            }
            if (gamepad1.a){
                System.out.println(robot.poseModule.getPos());
            }
            if (gamepad1.dpad_up){
                gamepad1.rumble(1.0,0,700);
            }
            if (gamepad1.dpad_down){
                gamepad2.rumble(1.0,0,700);
            }
            if (gamepad1.dpad_left){
                gamepad1.rumble(0,1.0,700);
            }
            if (gamepad1.dpad_right){
                gamepad2.rumble(0,1.0,700);
            }

            //==========================DRIVER TWO==================================================================

            if (Math.abs(robot.hwCollection.clawCoder.getEncoderPosition() - (ClawModule.CONE_ENCODER_DIFF + ClawModule.ClawState.OPEN.clawCoder)) < 100
                    && robot.clawModule.getClawState().equals("Close") && Math.abs(robot.hwCollection.clawCoder.getEncoderPosition() - prevClawCoder) == 0
                    && counter == 0){
                gamepad2.rumble(1.0,0,700);
                counter = 1;
            } else if(robot.clawModule.getClawState().equals("Open")){
                counter = 0;
            }
            prevClawCoder = robot.hwCollection.clawCoder.getEncoderPosition();
//            telemetry.addData("clawstate", robot.clawModule.getClawState());
            if (gamepad2.a) {
//                telemetry.addData("A is pressed", "");
//                robot.clawModule.setClose();
            }
            if (gamepad2.left_bumper) {
                robot.clawModule.setClose();
            }
            if (gamepad2.right_bumper) {
                robot.clawModule.setOpen();
            }
            if (gamepad2.b) {
//                telemetry.addData("B is pressed", "");
                robot.slidesModule.setTargetPositionPreset(SlidesModule.SlidePositionPreset.JUNC_4);
                robot.susanModule.goToCustomDeg(90);
//                robot.clawModule.setOpen();
            }
            if (gamepad2.x) {
                robot.slidesModule.setTargetPositionPreset(SlidesModule.SlidePositionPreset.JUNC_4);
                robot.susanModule.goToCustomDeg(-90);
            }
            if (gamepad2.y) {
                robot.susanModule.goToFront();

                robot.slidesModule.setTargetPositionPreset(SlidesModule.SlidePositionPreset.GROUND);
            }  else if (gamepad2.left_trigger > 0){
                robot.susanModule.setReset(robot.hwCollection.susanMotor1.getEncoder().getEncoderPosition());
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
            // ==================== MOVING AVERAGE ====================
            robot.movingAverageFilter.addOdoX();
            robot.movingAverageFilter.addOdoY();
            robot.movingAverageFilter.addOdoTheta();
//            robot.movingAverageFilter.addControlIMUTheta();
//            robot.mSonicModule.tripleThreat(robot.movingAverageFilter.getAverageX(),robot.movingAverageFilter.getAverageY(),robot.movingAverageFilter.getAverageTheta(),robot.hwCollection.ultraSonic1.getDistance(DistanceUnit.INCH),robot.hwCollection.ultraSonic2.getDistance(DistanceUnit.INCH),robot.hwCollection.ultraSonic3.getDistance(DistanceUnit.INCH));
            // ==================== TELEMETRY ====================
            if (Math.random() * 100 < 0.1) {
                telemetry.addData(ImageOutput.bryant,"");
            } else if (Math.random() * 100 < 0.2) {
                telemetry.addData(ImageOutput.isaac,"");
            } else {
                telemetry.addData("lift height", "%f in", robot.slidesModule.getCurPosition().getValue(Distance.Unit.INCHES));
                telemetry.addData("susan position", robot.hwCollection.susanMotor1.getEncoder().getEncoderPosition());
                telemetry.addData("loop frequency", "%dHz", robot.diagnosticModule.getLoopFrequencyHz());
                telemetry.addData("left encoder", robot.hwCollection.driveEncoderLeft.getEncoderPosition());
                telemetry.addData("right encoder", robot.hwCollection.driveEncoderRight.getEncoderPosition());
                telemetry.addData("center encoder", robot.hwCollection.driveEncoderCenter.getEncoderPosition());
                telemetry.addData("Position", robot.poseModule.getPos());
                telemetry.addData("x", robot.movingAverageFilter.getAverageX());
                telemetry.addData("y",  robot.movingAverageFilter.getAverageY());
                telemetry.addData("heading", robot.movingAverageFilter.getAverageTheta());
                telemetry.addData("Yaw", orientation.getYaw(AngleUnit.DEGREES));
                telemetry.addData("Pitch", orientation.getPitch(AngleUnit.DEGREES));
                telemetry.addData("Roll", orientation.getRoll(AngleUnit.DEGREES));
                telemetry.addData("ultraFront dist", robot.hwCollection.ultraFront.getDistance(DistanceUnit.INCH));
                telemetry.addData("ultraRight dist", robot.hwCollection.ultraRight.getDistance(DistanceUnit.INCH));
                telemetry.addData("ultraLeft dist", robot.hwCollection.ultraLeft.getDistance(DistanceUnit.INCH));
                telemetry.addData("ultraFront2 dist", robot.hwCollection.ultraFront2.getDistance(DistanceUnit.INCH));
                telemetry.addData("ultraRight2 dist", robot.hwCollection.ultraRight2.getDistance(DistanceUnit.INCH));
                telemetry.addData("ultraLeft2 dist", robot.hwCollection.ultraLeft2.getDistance(DistanceUnit.INCH));
//                telemetry.addData("dFront dist", robot.hwCollection.dFront.getDistance(DistanceUnit.INCH));
//                telemetry.addData("dLeft dist", robot.hwCollection.dLeft.getDistance(DistanceUnit.INCH));
//                telemetry.addData("dRight dist", robot.hwCollection.dRight.getDistance(DistanceUnit.INCH));
//                telemetry.addData("dBack dist", robot.hwCollection.dBack.getDistance(DistanceUnit.INCH));
                telemetry.addData("claw encoder", robot.hwCollection.clawCoder.getEncoderPosition());
//                telemetry.addData("IMU Yaw", robot.hwCollection.controlIMU.getYawDeg());
//                telemetry.addData("IMU Pitch", robot.hwCollection.controlIMU.getPitchDeg());
//                telemetry.addData("IMU Roll", robot.hwCollection.controlIMU.getRollDeg());
//                telemetry.addData("x", "%f in", robot.navModule.getPose().x.getValue(Distance.Unit.INCHES));
//                telemetry.addData("y", "%f in", robot.navModule.getPose().y.getValue(Distance.Unit.INCHES));
//                telemetry.addData("heading", "%f deg", robot.navModule.getHeading().getStandard(Angle.Unit.DEGREES));
//            telemetry.addData("ultrasonic1 dist", robot.hwCollection.ultraSonic1.getDistance(DistanceUnit.INCH));
//            telemetry.addData("ultrasonic2 dist", robot.hwCollection.ultraSonic2.getDistance(DistanceUnit.INCH));
//            telemetry.addData("ultrasonic3 dist", robot.hwCollection.ultraSonic3.getDistance(DistanceUnit.INCH));
//            telemetry.addData("ultrasonic pos", robot.mSonicModule.getPos());
//            telemetry.addData("ultrasonic pos", Arrays.toString(robot.mSonicModule.tripleThreat(
//                    robot.movingAverageFilter.getAverageX() * 39.37,
//                    robot.movingAverageFilter.getAverageY() * 39.37,
//                    robot.movingAverageFilter.getAverageTheta(),
//                    robot.hwCollection.ultraSonic1.getDistance(DistanceUnit.INCH),
//                    robot.hwCollection.ultraSonic2.getDistance(DistanceUnit.INCH),
//                    robot.hwCollection.ultraSonic3.getDistance(DistanceUnit.INCH))));
//            telemetry.addData("Ultrasonic 1", robot.hwCollection.ultraSonic1.getDistance(DistanceUnit.INCH));
//            telemetry.addData("Ultrasonic 2", robot.hwCollection.ultraSonic2.getDistance(DistanceUnit.INCH));
//            telemetry.addData("Ultrasonic 3", robot.hwCollection.ultraSonic3.getDistance(DistanceUnit.INCH));
            }
            telemetry.update();
        }
        robot.stopHardwareLoop();
    }
}
