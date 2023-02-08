package org.quixilver8404.powerplay.opmode.test;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.VoltageUnit;
import org.quixilver8404.powerplay.control.AutonRobot;
import org.quixilver8404.powerplay.control.BaseRobot;
import org.quixilver8404.powerplay.control.SlidesModule;
import org.quixilver8404.powerplay.control.TaskModule;
import org.quixilver8404.powerplay.util.Vector3;
import org.quixilver8404.powerplay.util.measurement.Angle;
import org.quixilver8404.powerplay.util.measurement.Distance;

@Autonomous(group = "Good")
public class LeftButGood extends LinearOpMode {
    int variant = 0;

    @Override
    public void runOpMode() throws InterruptedException {

        telemetry.addData("status", "initializing hardware...");
        telemetry.update();

        AutonRobot robot = new AutonRobot(new Vector3(17.5/2 * 0.0254,(141-15) *0.0254,-90),this);
        robot.startHardwareLoop();
        robot.headingLockModule.enablePID(robot);

        robot.hwCollection.camera.setPipeline(robot.cvTasksModule);


        while (opModeInInit()) {
            variant = robot.cvTasksModule.getVariant();
            telemetry.addData("Auton Var", variant);
            telemetry.addData("status", "ready!");
            telemetry.addData("left ultrasonic sensor", robot.hwCollection.ultraSonic1.getDistance(DistanceUnit.MM));
            telemetry.addData("right ultrasonic sensor", robot.hwCollection.ultraSonic2.getDistance(DistanceUnit.MM));
            telemetry.update();
        }
        robot.poseModule.setStartPos(new Vector3(17.5/2 * 0.0254,(141-robot.hwCollection.ultraSonic1.getDistance(DistanceUnit.INCH)) * 0.0254,0));

        // robot claw starts closed
        robot.clawModule.setClose();

        if (isStopRequested()) {
            robot.stopHardwareLoop();
            return;
        }
        robot.pidPositionEstimation.setPoint(new Vector3((17.5/2 + 12) * 0.0254,(141-robot.hwCollection.ultraSonic1.getDistance(DistanceUnit.INCH)) * 0.0254,0));
        robot.pidPositionEstimation.goXY();
//        robot.actions.pickUpPreload();
        while (opModeIsActive()) {
            telemetry.addData("loop frequency", "%dHz", robot.diagnosticModule.getLoopFrequencyHz());
            telemetry.addData("left encoder", robot.hwCollection.driveEncoderLeft.getEncoderPosition());
            telemetry.addData("right encoder", robot.hwCollection.driveEncoderRight.getEncoderPosition());
            telemetry.addData("center encoder", robot.hwCollection.driveEncoderCenter.getEncoderPosition());
            telemetry.addData("x", robot.movingAverageFilter.getAverageX());
            telemetry.addData("y",  robot.movingAverageFilter.getAverageY());
            telemetry.addData("heading", robot.movingAverageFilter.getAverageTheta());
            telemetry.addData("power settings", robot.hwCollection.driveMotorFL.getPower() + ", "
                    + robot.hwCollection.driveMotorFR.getPower() + ", "
                    + robot.hwCollection.driveMotorBL.getPower() + ", "
                    + robot.hwCollection.driveMotorBR.getPower());
            telemetry.update();
            System.out.println(robot.hwCollection.driveMotorFL.getPower() + ", "
                    + robot.hwCollection.driveMotorFR.getPower() + ", "
                    + robot.hwCollection.driveMotorBL.getPower() + ", "
                    + robot.hwCollection.driveMotorBR.getPower());
        }
    }
}
