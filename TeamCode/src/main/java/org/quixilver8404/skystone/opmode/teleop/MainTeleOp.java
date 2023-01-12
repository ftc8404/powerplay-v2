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

@TeleOp(name = "Main Tele-Op", group = "Main")
public class MainTeleOp extends LinearOpMode {

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
            robot.slidesModule.setTargetPower(-gamepad2.left_stick_y);
            robot.susanModule.setManualPower(gamepad2.right_stick_x);

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
            telemetry.addData("ultrasonic dist", robot.hwCollection.ultraSonic1.getDistance(DistanceUnit.INCH));
            telemetry.addData("ultrasonic pos", Arrays.toString(robot.mSonicModule.calcPos(
                    robot.navModule.getPose().x.getValue(Distance.Unit.INCHES),
                    robot.navModule.getPose().y.getValue(Distance.Unit.INCHES),
                    robot.navModule.getHeading().getStandard(Angle.Unit.RADIANS),
                    robot.hwCollection.ultraSonic1.getDistance(DistanceUnit.INCH),
                    (15 / 2 - 2.0), (17.5 / 2 - 2.625), -Math.PI / 2)));
            telemetry.update();
        }
        robot.stopHardwareLoop();
    }
}
