package org.quixilver8404.powerplay.opmode.test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.quixilver8404.powerplay.control.HardwareCollection;
import org.quixilver8404.powerplay.hardware.motor.EncoderlessMotor;
import org.quixilver8404.powerplay.hardware.sensor.Encoder;

//@TeleOp(group = "Test")
public class FreeOdometryTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        EncoderlessMotor driveMotorFL = new EncoderlessMotor("driveMotorFL", HardwareCollection.DRIVE_MOTOR_FL_DIRECTION, hardwareMap);
        EncoderlessMotor driveMotorFR = new EncoderlessMotor("driveMotorFR", HardwareCollection.DRIVE_MOTOR_FR_DIRECTION, hardwareMap);
        EncoderlessMotor driveMotorBL = new EncoderlessMotor("driveMotorBL", HardwareCollection.DRIVE_MOTOR_BL_DIRECTION, hardwareMap);
        EncoderlessMotor driveMotorBR = new EncoderlessMotor("driveMotorBR", HardwareCollection.DRIVE_MOTOR_BR_DIRECTION, hardwareMap);

        Encoder driveEncoderCenter = new Encoder(driveMotorFL, DcMotorSimple.Direction.FORWARD);
        Encoder driveEncoderRight = new Encoder(driveMotorBL, DcMotorSimple.Direction.REVERSE);
        Encoder driveEncoderLeft = new Encoder(driveMotorBR, DcMotorSimple.Direction.REVERSE);

        waitForStart();
        while (opModeIsActive()) {
            telemetry.addData("center", driveEncoderCenter.getEncoderPosition());
            telemetry.addData("right", driveEncoderRight.getEncoderPosition());
            telemetry.addData("left", driveEncoderLeft.getEncoderPosition());
            telemetry.update();

            if (gamepad1.a) {
                driveMotorFL.setPower(0.2);
                driveMotorFR.setPower(0.2);
                driveMotorBL.setPower(0.2);
                driveMotorBR.setPower(0.2);
            } else {
                driveMotorFL.setPower(0);
                driveMotorFR.setPower(0);
                driveMotorBL.setPower(0);
                driveMotorBR.setPower(0);
            }
        }
    }
}
