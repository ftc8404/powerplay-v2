package org.quixilver8404.powerplay.opmode.auton;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.quixilver8404.powerplay.control.AutonRobot;
import org.quixilver8404.powerplay.control.ClawModule;
import org.quixilver8404.powerplay.util.Vector3;

@Autonomous(group = "Good")
public class RightButGood extends LinearOpMode {
    int variant = 0;

    double posX = 63;
    double DRXstaxY = 125.5;
    double poleY = 91;
    double DRXstaxX = 59;

    double yPos;
    AutonRobot robot;

    @Override
    public void runOpMode() throws InterruptedException {

        telemetry.addData("status", "initializing hardware...");
        telemetry.update();

        robot = new AutonRobot(new Vector3(17.5/2 * 0.0254,-(15) * 0.0254,0),this);
        robot.startHardwareLoop();

        robot.headingLockModule.enablePID(robot);

        robot.hwCollection.camera.setPipeline(robot.cvTasksModule);

        while (opModeInInit()) {
            variant = robot.cvTasksModule.getVariant();
            telemetry.addData("Auton Var", variant);

            telemetry.update();
        }
        robot.actions.startTime();
        robot.clawModule.setOpen();
        robot.poseModule.setStartPos(new Vector3(17.5/2 * 0.0254,-(100) * 0.0254,0));

        if (isStopRequested()) {
            robot.stopHardwareLoop();
            return;
        }
        robot.pidPositionEstimation.setPoint(new Vector3(posX * 0.0254,-(100) * 0.0254,0));
        robot.pidPositionEstimation.goX();
        robot.actions.pickUpPreload(false);
        while (!robot.pidPositionEstimation.isNotMoving() && opModeIsActive() && robot.actions.shouldNotPark()){
            robot.pidPositionEstimation.goSmallTheta();
            telemetry();
        }
//        robot.pidPositionEstimation.goY();
        robot.pidPositionEstimation.setPoint(new Vector3(posX * 0.0254,-(100) * 0.0254,Math.PI/2 + Math.PI/6));
        robot.pidPositionEstimation.goTheta();
        while (!robot.pidPositionEstimation.isNotMoving() && opModeIsActive() && robot.actions.shouldNotPark()){
            telemetry();
        }
        robot.pidPositionEstimation.setPoint(new Vector3(posX * 0.0254,-(poleY) * 0.0254,Math.PI/2));
        robot.pidPositionEstimation.goHybridX(true);
        while (opModeIsActive() && robot.actions.isPickingUp() && robot.actions.shouldNotPark()){
//            System.out.println("preload:" + robot.actions.isPickingUp());
//            System.out.println("While loop: "+ (!robot.pidPositionEstimation.isNotMoving() && opModeIsActive() && robot.actions.isPickingUp()));
            robot.pidPositionEstimation.goHybridY(true);
            robot.pidPositionEstimation.goSmallTheta();
            telemetry();
        }
        System.out.println("preload:" + robot.actions.isPickingUp());
        robot.pidPositionEstimation.setPoint(new Vector3(DRXstaxX * 0.0254,-(DRXstaxY) * 0.0254,Math.PI/2 - 2 * Math.PI/180));
        robot.pidPositionEstimation.goHybridX(true);
        robot.actions.coneStackPickup(3.8, false);
        System.out.println("cone stack:" + robot.actions.isConeStackPickup());
        while (opModeIsActive() && robot.actions.isConeStackPickup() && robot.actions.shouldNotPark()){
            robot.pidPositionEstimation.goSmallTheta();
            robot.pidPositionEstimation.goHybridY(true);
            telemetry();
        }
        System.out.println("cone stack:" + robot.actions.isConeStackPickup());
        robot.pidPositionEstimation.setPoint(new Vector3((posX - 1) * 0.0254,-(poleY - 0.5) * 0.0254,Math.PI/2 - 2 * Math.PI/180));
        robot.pidPositionEstimation.goHybridX(true);
        while (!robot.pidPositionEstimation.isNotMoving() && opModeIsActive() && robot.actions.shouldNotPark()){
            robot.pidPositionEstimation.goSmallTheta();
            robot.pidPositionEstimation.goHybridY(true);
            telemetry();
        }
        robot.actions.open();
        while (robot.actions.isOpen() && opModeIsActive() && robot.actions.shouldNotPark()) {
            telemetry();
        }
        System.out.println("preload:" + robot.actions.isPickingUp());
        robot.pidPositionEstimation.setPoint(new Vector3((DRXstaxX + 0.1) * 0.0254,-((DRXstaxY) - 0.1) * 0.0254,Math.PI/2 - 8 * Math.PI/180));
        robot.pidPositionEstimation.goHybridX(true);
        robot.actions.coneStackPickup(3, false);
        System.out.println("cone stack:" + robot.actions.isConeStackPickup());
        while (opModeIsActive() && robot.actions.isConeStackPickup() && robot.actions.shouldNotPark()){
            robot.pidPositionEstimation.goSmallTheta();
            robot.pidPositionEstimation.goHybridY(true);
            telemetry();
        }
        System.out.println("cone stack:" + robot.actions.isConeStackPickup());
        robot.pidPositionEstimation.setPoint(new Vector3((posX - 2.9) * 0.0254,-(poleY - 0.7) * 0.0254,Math.PI/2 - 8 * Math.PI/180));
        robot.pidPositionEstimation.goHybridX(true);
        while (!robot.pidPositionEstimation.isNotMoving() && opModeIsActive() && robot.actions.shouldNotPark()){
            robot.pidPositionEstimation.goSmallTheta();
            robot.pidPositionEstimation.goHybridY(true);
            telemetry();
        }
        robot.actions.open();
        while(robot.actions.isOpen() && opModeIsActive()) {
            telemetry();
        }
        System.out.println("preload:" + robot.actions.isPickingUp());
        robot.pidPositionEstimation.setPoint(new Vector3((DRXstaxX - 0.5) * 0.0254,-((DRXstaxY)-0.2) * 0.0254,Math.PI/2 - 12 * Math.PI/180));
        robot.pidPositionEstimation.goHybridX(true);
        robot.actions.coneStackPickup(1.7, false);
        System.out.println("cone stack:" + robot.actions.isConeStackPickup());
        while (opModeIsActive() && robot.actions.isConeStackPickup() && robot.actions.shouldNotPark()){
            robot.pidPositionEstimation.goSmallTheta();
            robot.pidPositionEstimation.goHybridY(true);
            telemetry();
        }
        System.out.println("cone stack:" + robot.actions.isConeStackPickup());
        robot.pidPositionEstimation.setPoint(new Vector3((posX - 5.9) * 0.0254,-(poleY-0.3) * 0.0254,Math.PI/2 - 12 * Math.PI/180));
        robot.pidPositionEstimation.goHybridX(true);
        while (!robot.pidPositionEstimation.isNotMoving() && opModeIsActive() && robot.actions.shouldNotPark()){
            robot.pidPositionEstimation.goSmallTheta();
            robot.pidPositionEstimation.goHybridY(true);
            telemetry();
        }
        robot.actions.open();
        while(robot.actions.isOpen() && opModeIsActive() && robot.actions.shouldNotPark()) {
            telemetry();
        }
        if (variant == 1) {
            robot.pidPositionEstimation.setPoint(new Vector3((posX - 9.9) * 0.0254,-(77) * 0.0254,Math.PI/2 - 13 * Math.PI/180));
            robot.pidPositionEstimation.goHybridX(true);
            robot.actions.front();
            while (!robot.pidPositionEstimation.isNotMoving() && opModeIsActive()){
                robot.pidPositionEstimation.goSmallTheta();
                robot.pidPositionEstimation.goHybridY(true);
                telemetry();
            }
        } else if (variant == 2) {
            robot.pidPositionEstimation.setPoint(new Vector3((posX - 6.2) * 0.0254,-(100) * 0.0254,Math.PI/2 - 13 * Math.PI/180));
            robot.pidPositionEstimation.goHybridX(true);
            robot.actions.front();
            while (!robot.pidPositionEstimation.isNotMoving() && opModeIsActive()){
                robot.pidPositionEstimation.goSmallTheta();
                robot.pidPositionEstimation.goHybridY(true);
                telemetry();
            }
        } else if (variant == 3) {
            robot.pidPositionEstimation.setPoint(new Vector3((posX - 6.2) * 0.0254,-(125.5) * 0.0254,Math.PI/2 - 8 * Math.PI/180));
            robot.pidPositionEstimation.goHybridX(true);
            robot.actions.front();
            while (!robot.pidPositionEstimation.isNotMoving() && opModeIsActive()){
                robot.pidPositionEstimation.goSmallTheta();
                robot.pidPositionEstimation.goHybridY(true);
                telemetry();
            }
        }
//        final double newPosX = robot.movingAverageFilter.getAverageX();
//        final double newPosY = robot.movingAverageFilter.getAverageY();
//        while (!robot.actions.pickUp() && opModeIsActive()) {
//            telemetry();
//        }
//        System.out.println("newPosX: " + newPosX + "  newPosY: " + newPosY);
////        robot.movingAverageFilter.resetFilter();
//        robot.poseModule.setStartPos(new Vector3(17.5/2 * 0.0254,(141-yPos) * 0.0254,-Math.PI/2));
//        robot.pidPositionEstimation.setPoint(new Vector3(newPosX + (26) * 0.0254,newPosY + (5 * 0.0254),0));
//        System.out.println("Point but Left: " + robot.pidPositionEstimation.getPoint());
//        robot.pidPositionEstimation.goHybrid();
//        while (!robot.pidPositionEstimation.getMove() && opModeIsActive()){
//            telemetry();
//        }
//        robot.actions.dropCone();
//        while (!robot.actions.drop() && opModeIsActive()) {
//            telemetry();
//        }
//        robot.actions.park(variant);
        while (opModeIsActive()) {
            telemetry();
        }
    }

    public void telemetry() {
        telemetry.addData("pos", robot.poseModule.getPos().toString());
        telemetry.addData("curPosDeg", robot.susanModule.getCurPosDeg());
        telemetry.addData("susan power", robot.hwCollection.susanMotor1.getPower());
        telemetry.addData("loop frequency", "%dHz", robot.diagnosticModule.getLoopFrequencyHz());
        telemetry.addData("left encoder", robot.hwCollection.driveEncoderLeft.getEncoderPosition());
        telemetry.addData("right encoder", robot.hwCollection.driveEncoderRight.getEncoderPosition());
        telemetry.addData("center encoder", robot.hwCollection.driveEncoderCenter.getEncoderPosition());
        telemetry.addData("x", robot.movingAverageFilter.getAverageX() * 39.37);
        telemetry.addData("y", robot.movingAverageFilter.getAverageY() * 39.37);
        telemetry.addData("heading", robot.movingAverageFilter.getAverageTheta());
        telemetry.addData("claw encoder", robot.hwCollection.clawCoder.getEncoderPosition());
        telemetry.addData("claw close pos", ClawModule.ClawState.OPEN.clawCoder);
        telemetry.addData("claw state", robot.clawModule.getClawState());

//        telemetry.addData("ultrasonic1 dist", robot.hwCollection.ultraSonic1.getDistance(DistanceUnit.INCH));
//        telemetry.addData("ultrasonic2 dist", robot.hwCollection.ultraSonic2.getDistance(DistanceUnit.INCH));
//        telemetry.addData("ultrasonic3 dist", robot.hwCollection.ultraSonic3.getDistance(DistanceUnit.INCH));
//        telemetry.addData("ultrasonic pos", Arrays.toString(MSonicModule.tripleThreat(
//                robot.movingAverageFilter.getAverageX() * 39.37,
//                robot.movingAverageFilter.getAverageY() * 39.37,
//                robot.movingAverageFilter.getAverageTheta(),
//                robot.hwCollection.ultraSonic1.getDistance(DistanceUnit.INCH),
//                robot.hwCollection.ultraSonic2.getDistance(DistanceUnit.INCH),
//                robot.hwCollection.ultraSonic3.getDistance(DistanceUnit.INCH))));
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
