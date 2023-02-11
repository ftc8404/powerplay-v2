package org.quixilver8404.powerplay.opmode.test;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.VoltageUnit;
import org.quixilver8404.powerplay.control.AutonRobot;
import org.quixilver8404.powerplay.control.BaseRobot;
import org.quixilver8404.powerplay.control.MSonicModule;
import org.quixilver8404.powerplay.control.SlidesModule;
import org.quixilver8404.powerplay.control.TaskModule;
import org.quixilver8404.powerplay.util.Vector3;
import org.quixilver8404.powerplay.util.measurement.Angle;
import org.quixilver8404.powerplay.util.measurement.Distance;

import java.util.Arrays;

@Autonomous(group = "Good")
public class LeftButGood extends LinearOpMode {
    int variant = 0;
    double yPos;
    AutonRobot robot;

    @Override
    public void runOpMode() throws InterruptedException {

        telemetry.addData("status", "initializing hardware...");
        telemetry.update();

        robot = new AutonRobot(new Vector3(17.5/2 * 0.0254,(141-15) *0.0254,-90),this);
        robot.startHardwareLoop();
        robot.headingLockModule.enablePID(robot);

        robot.hwCollection.camera.setPipeline(robot.cvTasksModule);


        while (opModeInInit()) {
            variant = robot.cvTasksModule.getVariant();
            telemetry.addData("Auton Var", variant);
            telemetry.addData("status", "ready!");
            telemetry.addData("ultrasonic1 dist", robot.hwCollection.ultraSonic1.getDistance(DistanceUnit.INCH));
            telemetry.addData("ultrasonic2 dist", robot.hwCollection.ultraSonic2.getDistance(DistanceUnit.INCH));
            telemetry.addData("ultrasonic3 dist", robot.hwCollection.ultraSonic3.getDistance(DistanceUnit.INCH));
            yPos = robot.hwCollection.ultraSonic1.getDistance(DistanceUnit.INCH);
            telemetry.update();

        }
        System.out.println("Ypos" + yPos);
        robot.poseModule.setStartPos(new Vector3(17.5/2 * 0.0254,(141-yPos) * 0.0254,0));

        if (isStopRequested()) {
            robot.stopHardwareLoop();
            return;
        }
        robot.actions.dumpSignalCone();
        robot.pidPositionEstimation.setPoint(new Vector3((58.75) * 0.0254,(99) * 0.0254,Math.PI/2));
        robot.pidPositionEstimation.goX();
        while (!robot.pidPositionEstimation.getMove() && opModeIsActive()){
            telemetry();
        }
        robot.actions.pickUpPreload();
        robot.pidPositionEstimation.goTheta();
        while (!robot.pidPositionEstimation.getMove() && opModeIsActive()){
            telemetry();
        }
        final double newPosX = robot.movingAverageFilter.getAverageX();
        final double newPosY = robot.movingAverageFilter.getAverageY();
        while (!robot.actions.pickUp() && opModeIsActive()) {
            telemetry();
        }
        System.out.println("newPosX: " + newPosX + "  newPosY: " + newPosY);
//        robot.movingAverageFilter.resetFilter();
        robot.poseModule.setStartPos(new Vector3(17.5/2 * 0.0254,(141-yPos) * 0.0254,-Math.PI/2));
        robot.pidPositionEstimation.setPoint(new Vector3(newPosX + (26) * 0.0254,newPosY + (5 * 0.0254),0));
        System.out.println("Point but Left: " + robot.pidPositionEstimation.getPoint());
        robot.pidPositionEstimation.goHybrid();
        while (!robot.pidPositionEstimation.getMove() && opModeIsActive()){
            telemetry();
        }
        robot.actions.dropCone();
        while (!robot.actions.drop() && opModeIsActive()) {
            telemetry();
        }
        robot.actions.park(variant);
        while (opModeIsActive()) {
            telemetry();
        }
    }

    public void telemetry() {
        telemetry.addData("curPosDeg", robot.susanModule.getCurPosDeg());
        telemetry.addData("susan power", robot.hwCollection.susanMotor1.getPower());
        telemetry.addData("loop frequency", "%dHz", robot.diagnosticModule.getLoopFrequencyHz());
        telemetry.addData("left encoder", robot.hwCollection.driveEncoderLeft.getEncoderPosition());
        telemetry.addData("right encoder", robot.hwCollection.driveEncoderRight.getEncoderPosition());
        telemetry.addData("center encoder", robot.hwCollection.driveEncoderCenter.getEncoderPosition());
        telemetry.addData("x", robot.movingAverageFilter.getAverageX() * 39.37);
        telemetry.addData("y", robot.movingAverageFilter.getAverageY() * 39.37);
        telemetry.addData("heading", robot.movingAverageFilter.getAverageTheta());
        telemetry.addData("ultrasonic1 dist", robot.hwCollection.ultraSonic1.getDistance(DistanceUnit.INCH));
        telemetry.addData("ultrasonic2 dist", robot.hwCollection.ultraSonic2.getDistance(DistanceUnit.INCH));
        telemetry.addData("ultrasonic3 dist", robot.hwCollection.ultraSonic3.getDistance(DistanceUnit.INCH));
        telemetry.addData("ultrasonic pos", Arrays.toString(MSonicModule.tripleThreat(
                robot.movingAverageFilter.getAverageX() * 39.37,
                robot.movingAverageFilter.getAverageY() * 39.37,
                robot.movingAverageFilter.getAverageTheta(),
                robot.hwCollection.ultraSonic1.getDistance(DistanceUnit.INCH),
                robot.hwCollection.ultraSonic2.getDistance(DistanceUnit.INCH),
                robot.hwCollection.ultraSonic3.getDistance(DistanceUnit.INCH))));
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
