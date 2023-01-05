package org.quixilver8404.powerplaycode.opmodes;

import android.content.res.Resources;
import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.R;
import org.quixilver8404.powerplaycode.control.base.Robot;
import org.quixilver8404.powerplaycode.control.base.modules.PositionTrackingModule;
import org.quixilver8404.powerplaycode.util.Vector3;

import java.io.IOException;
import java.util.Arrays;

import org.openftc.easyopencv.OpenCvWebcam;
import org.quixilver8404.powerplaycode.control.base.modules.CVTasksModule;
@Autonomous(name = "NonBasic Left Auton", group = "Main")
public class NonBasic_Left_Auton extends LinearOpMode {

    static final int STREAM_WIDTH = 1920; // modify for your camera
    static final int STREAM_HEIGHT = 1080; // modify for your camera
    OpenCvWebcam webcam;
    CVTasksModule pipeline;

    @Override
    public void runOpMode() throws InterruptedException {

        final double inToM = 0.0254;

        telemetry.addLine("========== Autonomous ==========");
        telemetry.addData("Status", "initializing Breakout...");
        telemetry.update();

        final Robot robot = new Robot(new Vector3(8.75*inToM, 104.5*inToM,-Math.PI/2), this);
//        robot.poseModule.setAction(PositionTrackingModule.PositionTrackingAction.ENABLE_ASYNC_UPDATES);
        robot.poseModule.setAction(PositionTrackingModule.PositionTrackingAction.SWITCH_TO_ODOMETRY);

        final Resources resources = hardwareMap.appContext.getResources();
        robot.breakoutModule.init(resources.openRawResource(R.raw.nonbasic_auton_left));
        Log.d("Breakout Status","Work'd");
        robot.breakoutModule.stop();
        robot.startHardwareLoop();

        robot.hardwareCollection.camera.setPipeline(robot.cvTasksModule);
        robot.breakoutModule.setVariant(robot.cvTasksModule.getVariant());

        while (!isStarted() && !isStopRequested()) {
            telemetry.addLine("========== Autonomous ==========");
            telemetry.addData("Variant", robot.cvTasksModule.getVariant());
            telemetry.addData("Blue Counter", robot.cvTasksModule.finalBlueCounter);
            telemetry.addData("Green Counter", robot.cvTasksModule.finalGreenCounter);
            telemetry.addData("Red Counter", robot.cvTasksModule.finalRedCounter);
            telemetry.addData("Status", "ready!");
            telemetry.update();
        }

        robot.breakoutModule.resume();
        robot.driveModule.enableAuton();

        while (opModeIsActive()) {
            Log.d("diff", "this is new");
            telemetry.addData("Movement mode", robot.driveModule.getMovementMode());
            telemetry.addData("Power settings", Arrays.toString(robot.driveModule.getPowerSettings()));
            telemetry.update();
        }
        if (isStopRequested()) {
            robot.breakoutModule.stop();
            robot.breakoutModule.bytes = null;
            robot.stopHardwareLoop();
        }
    }
}
