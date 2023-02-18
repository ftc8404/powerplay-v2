package org.quixilver8404.powerplay.opmode.test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.R;
import org.quixilver8404.foxtrot.FoxtrotPath;
import org.quixilver8404.powerplay.control.ActionSet;
import org.quixilver8404.powerplay.control.AutonRobot;
import org.quixilver8404.powerplay.control.BaseRobot;
import org.quixilver8404.powerplay.control.SlidesModule;
import org.quixilver8404.powerplay.control.TaskModule;
import org.quixilver8404.powerplay.util.measurement.Angle;
import org.quixilver8404.powerplay.util.measurement.Distance;

import java.io.IOException;

//@Autonomous(group = "Test")
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

//        robot.hwCollection.camera.setPipeline(robot.cvTasksModule);

        int variant;

        while (opModeInInit()) {
            variant = robot.cvTasksModule.getVariant();
            telemetry.addData("Auton Var", variant);
            telemetry.addData("status", "ready!");
            telemetry.addData("left ultrasonic sensor", robot.hwCollection.ultraSonic1.getDistance(DistanceUnit.MM));
            telemetry.addData("right ultrasonic sensor", robot.hwCollection.ultraSonic2.getDistance(DistanceUnit.MM));
            telemetry.update();
        }

        // robot claw starts closed
        robot.clawModule.setClose();

        if (isStopRequested()) {
            robot.stopHardwareLoop();
            return;
        }

        robot.navModule.setPosAndHeadingToPathStart(foxtrotPath, robot);
        robot.taskModule.addTask(new TaskModule.Task() {
            @Override
            public boolean loop(int runningTimeMillis, BaseRobot baseRobot) {
                if (runningTimeMillis < 1500) {
                    baseRobot.clawModule.setClose();
                    return false;
                } else if (runningTimeMillis < 2000) {
                    baseRobot.slidesModule.setTargetPositionPreset(SlidesModule.SlidePositionPreset.JUNC_1);
                    return false;
                } else {
                    robot.pathFollowModule.follow(foxtrotPath.getPPPath(), actionSet, robot);
                    return true;
                }
            }
        });
        while (opModeIsActive()) {
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
