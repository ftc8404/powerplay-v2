package org.quixilver8404.skystone.opmode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.quixilver8404.skystone.control.AutonRobot;
import org.quixilver8404.skystone.util.measurement.Angle;
import org.quixilver8404.skystone.util.measurement.Distance;

@TeleOp(name = "Main Tele-Op", group = "Main")
public class MainTeleOp extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("status", "initializing hardware...");
        telemetry.update();

        final AutonRobot robot = new AutonRobot(this);

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
            // ==================== TELEMETRY ====================
            // TODO remove after testing
            telemetry.addData("loop frequency", "%dHz", robot.diagnosticModule.getLoopFrequencyHz());
            telemetry.addData("left encoder", robot.hwCollection.driveEncoderLeft.getEncoderPosition());
            telemetry.addData("right encoder", robot.hwCollection.driveEncoderRight.getEncoderPosition());
            telemetry.addData("center encoder", robot.hwCollection.driveEncoderCenter.getEncoderPosition());
            telemetry.addData("x", "%f in", robot.navModule.getPose().x.getValue(Distance.Unit.INCHES));
            telemetry.addData("y", "%f in", robot.navModule.getPose().y.getValue(Distance.Unit.INCHES));
            telemetry.addData("heading", "%f deg", robot.navModule.getHeading().getStandard(Angle.Unit.DEGREES));
            telemetry.addData("ultrasonic pos",robot.mSonicModule.calcPos(
                    robot.navModule.getPose().x.getValue(Distance.Unit.INCHES),
                    robot.navModule.getPose().y.getValue(Distance.Unit.INCHES),
                    robot.navModule.getHeading().getStandard(Angle.Unit.RADIANS),
                    robot.hwCollection.ultraSonic1.getDistance(DistanceUnit.INCH),
                    15/2-2.0,17.5/2 -2.625,-Math.PI/2
            ) );
            telemetry.update();
        }
        robot.stopHardwareLoop();
    }
}
