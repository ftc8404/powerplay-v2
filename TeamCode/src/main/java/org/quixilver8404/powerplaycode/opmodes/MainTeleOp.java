package org.quixilver8404.powerplaycode.opmodes;

import android.util.Log;

import java.util.Objects;
import java.util.Random;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.quixilver8404.powerplaycode.control.base.Robot;
import org.quixilver8404.powerplaycode.control.base.modules.SlideModule;
import org.quixilver8404.powerplaycode.control.base.modules.SusanModule;
import org.quixilver8404.powerplaycode.util.Angles;
import org.quixilver8404.powerplaycode.util.Vector3;

@TeleOp(name = "Main Tele-Op", group = "Main")
public class MainTeleOp extends OpMode {

    public Byte r;
    public Byte g;
    public Byte b;

    private final ElapsedTime runtime = new ElapsedTime();

    // velocity to drive the robot when the trigger is not pressed
    public static final double BASE_DRIVE_POWER = 0.7;
    public static final double BASE_ROTATE_POWER = 0.7;

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

    public static boolean frontFlipped = false;
    public static boolean flipFrontReleased = true;
    public static Robot robot;

    public static double turret;
    public static double lift;

    public static float liftheight;

    public static boolean ttp1;
    public static boolean ttp2;
    public static boolean ttp3;
    public static boolean ttp4;

    public static boolean liftp1;
    public static boolean liftp2;
    public static boolean liftp3;
    public static boolean liftp4;

    public static boolean clawReleased = true;
    public static boolean clawOpened = false;

    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");
        System.out.println("init");

        robot = new Robot(new Vector3(), this);
        robot.startHardwareLoop();
        robot.clawModule.setClose();
    }

    @Override
    public void init_loop() {
        System.out.println("init loop beep boop");
        telemetry.addData("ultraSonic distance", robot.hardwareCollection.ultraSonic1.getDistance(DistanceUnit.INCH));
    }

    @Override
    public void start() {
        System.out.println("start");
        runtime.reset();
    }

    @Override
    public void loop() {
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        System.out.println("loop");
        //add telemetry stuffs here
        double x = gamepad1.left_stick_x;
        // For some reason, the controllers or sdk or whatever flips up with down ¯\_(ツ)_/¯
        double y = -gamepad1.left_stick_y;

        telemetry.addData("x", x);
        telemetry.addData("y", y);

        Log.d("Gamepad_y", "" + y);

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
            angle = Angles.zeroToPi(angle + Math.PI);
        }

        angle = Angles.zeroToPi(angle + Math.PI/2);

        // moving the joystick right should give a negative rotation (clockwise)
        double rotatePower = -gamepad1.right_stick_x;
        rotatePower = Math.signum(rotatePower)*Math.sqrt(Math.abs(rotatePower));

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


        double dpadPowerForward = 0;
        double dpadPowerStrafe = 0;
        double dpadPowerRotate = 0;

        boolean dpadForMovement = gamepad1.a;

        if (gamepad1.dpad_up) {
            dpadPowerForward += DPAD_DRIVE_POWER_FORWARD;
        }
        if (gamepad1.dpad_down) {
            dpadPowerForward -= DPAD_DRIVE_POWER_FORWARD;
        }
        if (dpadForMovement) {
            if (gamepad1.dpad_left) {
                dpadPowerStrafe += DPAD_DRIVE_POWER_STRAFE;
            }
            if (gamepad1.dpad_right) {
                dpadPowerStrafe -= DPAD_DRIVE_POWER_STRAFE;
            }
        } else {
            if (gamepad1.dpad_left) {
                dpadPowerRotate += DPAD_DRIVE_POWER_ROTATE;
            }
            if (gamepad1.dpad_right) {
                dpadPowerRotate -= DPAD_DRIVE_POWER_ROTATE;
            }
        }

        if (frontFlipped) {
            dpadPowerForward *= -1;
            dpadPowerStrafe *= -1;
        }

        if (dpadPowerForward != 0 || dpadPowerStrafe != 0 || dpadPowerRotate != 0) {
            robot.driveModule.setIntrinsicTargetPower(dpadPowerStrafe, dpadPowerForward);
            robot.driveModule.setTargetRotatePower(dpadPowerRotate);
            telemetry.addData("Intrinsic target power", ""+dpadPowerForward+", "+dpadPowerStrafe);
            telemetry.addData("Rotate target power", ""+dpadPowerRotate);
        } else {
            robot.driveModule.setIntrinsicTargetVector(magnitude, angle);
            robot.driveModule.setTargetRotatePower(rotatePower);
            telemetry.addData("Intrinsic target vector", ""+magnitude+", "+angle);
            telemetry.addData("Rotate target power", ""+rotatePower);
        }

        telemetry.addData("FR", robot.hardwareCollection.driveMotorFR.getPower());
        telemetry.addData("FL", robot.hardwareCollection.driveMotorFL.getPower());
        telemetry.addData("BL", robot.hardwareCollection.driveMotorBL.getPower());
        telemetry.addData("BR", robot.hardwareCollection.driveMotorBR.getPower());
        telemetry.addData("Position", robot.poseModule.getPos());

        // handle flipping front
        if (gamepad1.y) {
            if (flipFrontReleased) {
                frontFlipped = !frontFlipped;
                flipFrontReleased = false;
            }
        } else {
            flipFrontReleased = true;
        }

        //========================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================
        //Driver Two
        //========================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================

        //left joystick: lift
        lift = -gamepad2.left_stick_y;

        if (robot.slideModule.getSlideControl() == SlideModule.SlideControlState.MANUAL || lift != 0) {
            robot.slideModule.setManualPower(lift);
        }


        //right joystick: turret
        turret = gamepad2.right_stick_x;
        if (robot.susanModule.getSusanControl() == SusanModule.SusanControlState.MANUAL || turret != 0) {
            robot.susanModule.setManualPower(turret);
        }

        telemetry.addData("Slides1 Power", robot.hardwareCollection.slidesMotor1.getPower());
        telemetry.addData("Slides2 Power", robot.hardwareCollection.slidesMotor2.getPower());
        telemetry.addData("Slides Position", robot.slideModule.position);
        telemetry.addData("Slide State", robot.slideModule.getSlideState());
        telemetry.addData("Slide Action", robot.slideModule.getSlideAction());
        telemetry.addData("Slide Control", robot.slideModule.getSlideControl());


        telemetry.addData("Susan1 Power", robot.hardwareCollection.susanMotor1.getPower());
        telemetry.addData("Susan2 Power", robot.hardwareCollection.susanMotor2.getPower());
        telemetry.addData("Susan Position", robot.susanModule.position);
        telemetry.addData("Susan State", robot.susanModule.getSusanState());
        telemetry.addData("Susan Action", robot.susanModule.getSusanAction());
        telemetry.addData("Susan Control", robot.susanModule.getSusanControl());

        telemetry.addData("Robot Position", robot.poseModule.getPos());


        //dpad: claw (lift presets)


//        liftp1 = gamepad2.dpad_left;
//        liftp2 = gamepad2.dpad_up;
//        liftp3 = gamepad2.dpad_right;
//        liftp4 = gamepad2.dpad_down;

        //letter buttons: turret presets

        ttp1 = gamepad2.x;
        if (ttp1){
//            robot.slideModule.goToJunc4();
            robot.susanModule.goToFront();
        }

        ttp2 = gamepad2.y;
        if (ttp2){
            robot.slideModule.goToJunc3();
        }

        ttp3 = gamepad2.b;
        if (ttp3){
            robot.slideModule.goToJunc2();
        }

        ttp4 = gamepad2.a;
        if (ttp4){
//            robot.susanModule.setDesiredpos();
        }

        // bumper: cycle claw positions


        // claw
        telemetry.addData("clawState", robot.clawModule.getClawState());

        if(gamepad2.left_trigger >= 0.5){
            robot.clawModule.setClose();
        } else {
            robot.clawModule.setOpen();
            if (gamepad2.left_bumper) {
                if (clawReleased) {
                    clawOpened = !clawOpened;
                    clawReleased = false;
                }
            } else {
                clawReleased = true;
            }
            if (clawOpened){
                robot.clawModule.setOpen();
            } else {
                robot.clawModule.setClose();
            }
        }
    }

    @Override
    public void stop() {
        System.out.println("stop");
        robot.hardwareCollection.susanMotor1.setPower(0);
        robot.hardwareCollection.susanMotor2.setPower(0);
        robot.hardwareCollection.slidesMotor1.setPower(0);
        robot.hardwareCollection.slidesMotor2.setPower(0);
        try {
            robot.stopHardwareLoop();
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    }

}