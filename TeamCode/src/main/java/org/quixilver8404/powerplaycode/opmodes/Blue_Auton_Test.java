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
import java.io.InputStream;
import java.util.Arrays;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;
import org.quixilver8404.powerplaycode.control.base.HardwareCollection;
import org.quixilver8404.powerplaycode.control.base.Robot;
import org.quixilver8404.powerplaycode.control.base.modules.CVTasksModule;
 @Autonomous(name = "Blue_Test", group = "Main")
public class Blue_Auton_Test extends LinearOpMode {

     static final int STREAM_WIDTH = 1920; // modify for your camera
     static final int STREAM_HEIGHT = 1080; // modify for your camera
     OpenCvWebcam webcam;
     CVTasksModule pipeline;

    @Override
    public void runOpMode() throws InterruptedException {

        final double inToM = 0.0254;

        telemetry.addLine("========== Autonomous ==========");
        telemetry.addData("status", "initializing Breakout...");
        telemetry.update();

        final Robot robot = new Robot(new Vector3(4.32*inToM, 105.68*inToM,-Math.PI/2), this); //Starting position for both carousel autons
//        robot.poseModule.setAction(PositionTrackingModule.PositionTrackingAction.ENABLE_ASYNC_UPDATES);
        robot.poseModule.setAction(PositionTrackingModule.PositionTrackingAction.SWITCH_TO_ODOMETRY);


        final Resources resources = hardwareMap.appContext.getResources();
//        try {
////            robot.breakoutModule.init(resources.openRawResource(R.raw.killruhan));
//        } catch (IOException e) {
//            e.printStackTrace();
//        }
        robot.breakoutModule.stop();
        robot.startHardwareLoop();

        while (!isStarted() && !isStopRequested()) {
            //CV sample while init not moving
            int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
            WebcamName webcamName = null;
            webcamName = hardwareMap.get(WebcamName.class, "webcam"); // put your camera's name here
            webcam = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);
            pipeline = new CVTasksModule();
            webcam.setPipeline(pipeline);
            webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
            {
                @Override
                public void onOpened()
                {
                    webcam.startStreaming(STREAM_WIDTH, STREAM_HEIGHT, OpenCvCameraRotation.UPRIGHT);
                }

                @Override
                public void onError(int errorCode) {
                    telemetry.addData("Camera Failed","");
                    telemetry.update();
                }
            });

            telemetry.addLine("\n========== Autonomous ==========");
            telemetry.addData("status", "ready!");
            telemetry.addData("Color", "red");//color form cv
            telemetry.update();
        }
        Log.d("HI","----------Before Resume");
        robot.breakoutModule.setVariant(0);
        robot.breakoutModule.resume();
        robot.breakoutModule.setVariant(0);
        Log.d("HI","----------After Resume");
        //hi
        if (isStopRequested()) {
            robot.stopHardwareLoop();
            return;
        }


        robot.driveModule.enableAuton();

        while (opModeIsActive()) {
            Log.d("diff", "this is new");
            telemetry.addData("Movement mode", robot.driveModule.getMovementMode());
            telemetry.addData("power settings", Arrays.toString(robot.driveModule.getPowerSettings()));
            telemetry.update();
        }
        if (isStopRequested()) {
            robot.breakoutModule.stop();
            robot.breakoutModule.bytes = null;
            robot.stopHardwareLoop();
            return;
        }
    }
}
