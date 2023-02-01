package org.quixilver8404.powerplay.opmode.test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.VoltageUnit;
import org.quixilver8404.powerplay.control.AutonRobot;
import org.quixilver8404.powerplay.control.BaseRobot;
import org.quixilver8404.powerplay.control.SlidesModule;
import org.quixilver8404.powerplay.control.TaskModule;
import org.quixilver8404.powerplay.util.measurement.Angle;
import org.quixilver8404.powerplay.util.measurement.Distance;

@Autonomous(group = "Test")
public class LeftAuton extends LinearOpMode {
    int variant = 0;

    @Override
    public void runOpMode() throws InterruptedException {

        telemetry.addData("status", "initializing hardware...");
        telemetry.update();

        AutonRobot robot = new AutonRobot(this);
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

        // robot claw starts closed
        robot.clawModule.setClose();

        if (isStopRequested()) {
            robot.stopHardwareLoop();
            return;
        }
        double voltage = 13/robot.hwCollection.controlHub.getInputVoltage(VoltageUnit.VOLTS);
        robot.taskModule.addTask(new TaskModule.Task() {
            @Override
            public boolean loop(int runningTimeMillis, BaseRobot baseRobot) {
                if (runningTimeMillis < 1000) {
                    baseRobot.hwCollection.driveMotorFL.setPower(-0.5 * voltage);
                    baseRobot.hwCollection.driveMotorFR.setPower(-0.5 * voltage);
                    baseRobot.hwCollection.driveMotorBL.setPower(-0.5 * voltage);
                    baseRobot.hwCollection.driveMotorBR.setPower(-0.5 * voltage);
                    return false;
                } else if (runningTimeMillis < 1500) {
                    baseRobot.clawModule.setClose();
                    return false;
                } else if (runningTimeMillis < 2000) {
                    baseRobot.slidesModule.setTargetPositionPreset(SlidesModule.SlidePositionPreset.JUNC_2);
                    return false;
                } else if (runningTimeMillis < 4230) {
                    baseRobot.hwCollection.driveMotorFL.setPower(0.75 * voltage);
                    baseRobot.hwCollection.driveMotorFR.setPower(0.75 * voltage);
                    baseRobot.hwCollection.driveMotorBL.setPower(0.75 * voltage);
                    baseRobot.hwCollection.driveMotorBR.setPower(0.75 * voltage);
                    return false;
                } else if (runningTimeMillis < 6000) {
                    baseRobot.slidesModule.setTargetPositionPreset(SlidesModule.SlidePositionPreset.JUNC_4);
                    return false;
                } else if (runningTimeMillis < 6500) {
                    baseRobot.susanModule.goToCustomDeg(90);
                    return false;
                } else if (runningTimeMillis < 7550) {
                    baseRobot.headingLockModule.setTargetHeading(new Angle(90, Angle.Unit.DEGREES));
//                    baseRobot.hwCollection.driveMotorFL.setPower(-1 * voltage);
//                    baseRobot.hwCollection.driveMotorFR.setPower(1 * voltage);
//                    baseRobot.hwCollection.driveMotorBL.setPower(-1 * voltage);
//                    baseRobot.hwCollection.driveMotorBR.setPower(1 * voltage);
                    return false;
                } else if (runningTimeMillis < 8120) {
                    baseRobot.hwCollection.driveMotorFL.setPower(-0.75 * voltage);
                    baseRobot.hwCollection.driveMotorFR.setPower(-0.75 * voltage);
                    baseRobot.hwCollection.driveMotorBL.setPower(-0.75 * voltage);
                    baseRobot.hwCollection.driveMotorBR.setPower(-0.75 * voltage);
                    return false;
                } else if (runningTimeMillis < 8400) {
                    baseRobot.hwCollection.driveMotorFL.setPower(0);
                    baseRobot.hwCollection.driveMotorFR.setPower(0);
                    baseRobot.hwCollection.driveMotorBL.setPower(0);
                    baseRobot.hwCollection.driveMotorBR.setPower(0);
                    return false;
                } else if (runningTimeMillis < 8900) {
                    baseRobot.clawModule.setOpen();
                    return false;
                } else if (runningTimeMillis < 13000) {
                    baseRobot.susanModule.goToFront();
                    baseRobot.hwCollection.driveMotorFL.setPower(0.4 * voltage);
                    baseRobot.hwCollection.driveMotorFR.setPower(0.4 * voltage);
                    baseRobot.hwCollection.driveMotorBL.setPower(0.4 * voltage);
                    baseRobot.hwCollection.driveMotorBR.setPower(0.4 * voltage);
                    return false;
                } else if (runningTimeMillis < 13750) {
                    baseRobot.hwCollection.driveMotorFL.setPower(-0.5 * voltage);
                    baseRobot.hwCollection.driveMotorFR.setPower(-0.5 * voltage);
                    baseRobot.hwCollection.driveMotorBL.setPower(-0.5 * voltage);
                    baseRobot.hwCollection.driveMotorBR.setPower(-0.5 * voltage);
                    return false;
                } else if (runningTimeMillis < 15000) {
                    baseRobot.slidesModule.setTargetPosition(new Distance(110, Distance.Unit.MILLIMETERS));
                    baseRobot.hwCollection.driveMotorFL.setPower(0.3 * voltage);
                    baseRobot.hwCollection.driveMotorFR.setPower(0.3 * voltage);
                    baseRobot.hwCollection.driveMotorBL.setPower(0.3 * voltage);
                    baseRobot.hwCollection.driveMotorBR.setPower(0.3 * voltage);
                    return false;
                } else if (runningTimeMillis < 16000) {
                    baseRobot.hwCollection.driveMotorFL.setPower(0);
                    baseRobot.hwCollection.driveMotorFR.setPower(0);
                    baseRobot.hwCollection.driveMotorBL.setPower(0);
                    baseRobot.hwCollection.driveMotorBR.setPower(0);
                    baseRobot.clawModule.setClose();
                    return false;
                } else if (runningTimeMillis < 17000) {
                    baseRobot.slidesModule.setTargetPositionPreset(SlidesModule.SlidePositionPreset.JUNC_2);
                    return false;
                } else if (runningTimeMillis < 19400) {
                    baseRobot.hwCollection.driveMotorFL.setPower(-0.5 * voltage);
                    baseRobot.hwCollection.driveMotorFR.setPower(-0.5 * voltage);
                    baseRobot.hwCollection.driveMotorBL.setPower(-0.5 * voltage);
                    baseRobot.hwCollection.driveMotorBR.setPower(-0.5 * voltage);
                    baseRobot.slidesModule.setTargetPositionPreset(SlidesModule.SlidePositionPreset.JUNC_4);
                    return false;
                } else if (runningTimeMillis < 20500) {
                    baseRobot.susanModule.goToCustomDeg(90);
                    return false;
                } else if (runningTimeMillis < 21500) {
                    baseRobot.hwCollection.driveMotorFL.setPower(0);
                    baseRobot.hwCollection.driveMotorFR.setPower(0);
                    baseRobot.hwCollection.driveMotorBL.setPower(0);
                    baseRobot.hwCollection.driveMotorBR.setPower(0);
                    baseRobot.clawModule.setOpen();
                    return false;
                } else if (runningTimeMillis < 25600) {
                    baseRobot.susanModule.goToFront();
                    if (variant == 1){
                        baseRobot.hwCollection.driveMotorFL.setPower(0.4);
                        baseRobot.hwCollection.driveMotorFR.setPower(0.4);
                        baseRobot.hwCollection.driveMotorBL.setPower(0.4);
                        baseRobot.hwCollection.driveMotorBR.setPower(0.4);
                    } else if (variant == 2) {
                        baseRobot.hwCollection.driveMotorFL.setPower(0.18);
                        baseRobot.hwCollection.driveMotorFR.setPower(0.18);
                        baseRobot.hwCollection.driveMotorBL.setPower(0.18);
                        baseRobot.hwCollection.driveMotorBR.setPower(0.18);
                    } else {
                        baseRobot.hwCollection.driveMotorFL.setPower(-0.18);
                        baseRobot.hwCollection.driveMotorFR.setPower(-0.18);
                        baseRobot.hwCollection.driveMotorBL.setPower(-0.18);
                        baseRobot.hwCollection.driveMotorBR.setPower(-0.18);
                    }
                    return false;
                } else {
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
