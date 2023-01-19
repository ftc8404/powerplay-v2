package org.quixilver8404.skystone.opmode.test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.R;
import org.quixilver8404.foxtrot.FoxtrotPath;
import org.quixilver8404.skystone.control.ActionSet;
import org.quixilver8404.skystone.control.AutonRobot;
import org.quixilver8404.skystone.util.measurement.Angle;
import org.quixilver8404.skystone.util.measurement.Distance;

import java.io.IOException;

@Autonomous(group = "Test")
public class TestAuton extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        FoxtrotPath foxtrotPath;
        try {
            foxtrotPath = new FoxtrotPath(hardwareMap.appContext.getResources().openRawResource(R.raw.left_auton), 0);
        } catch (IOException e) {
            e.printStackTrace();
            return;
        }
        ActionSet actionSet = foxtrotPath.getActionSet();

        telemetry.addData("status", "initializing hardware...");
        telemetry.update();

        AutonRobot robot = new AutonRobot(this);
        robot.headingLockModule.disablePID();
        robot.startHardwareLoop();

        robot.hwCollection.camera.setPipeline(robot.cvTasksModule);

        int variant;

        while (opModeInInit()) {
            variant = robot.cvTasksModule.getVariant();
            telemetry.addData("Auton Var", variant);
            telemetry.addData("status", "ready!");
            telemetry.update();
        }

        if (isStopRequested()) {
            robot.stopHardwareLoop();
            return;
        }

        robot.navModule.setPosAndHeadingToPathStart(foxtrotPath, robot);
        robot.pathFollowModule.follow(foxtrotPath.getPPPath(), actionSet, robot);
        while (opModeIsActive() && robot.pathFollowModule.isBusy()) {
            idle();

            telemetry.addData("loop frequency", "%dHz", robot.diagnosticModule.getLoopFrequencyHz());
            telemetry.addData("left encoder", robot.hwCollection.driveEncoderLeft.getEncoderPosition());
            telemetry.addData("right encoder", robot.hwCollection.driveEncoderRight.getEncoderPosition());
            telemetry.addData("center encoder", robot.hwCollection.driveEncoderCenter.getEncoderPosition());
            telemetry.addData("x", "%f in", robot.navModule.getPose().x.getValue(Distance.Unit.INCHES));
            telemetry.addData("y", "%f in", robot.navModule.getPose().y.getValue(Distance.Unit.INCHES));
            telemetry.addData("heading", "%f deg", robot.navModule.getHeading().getStandard(Angle.Unit.DEGREES));
            telemetry.update();
        }
        robot.stopHardwareLoop();
        robot.stopDriveMotors();
    }
}
