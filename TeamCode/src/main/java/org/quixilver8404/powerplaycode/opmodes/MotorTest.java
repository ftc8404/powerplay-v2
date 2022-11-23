package org.quixilver8404.powerplaycode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.quixilver8404.powerplaycode.control.base.HardwareCollection;
import org.quixilver8404.powerplaycode.control.base.Robot;
import org.quixilver8404.powerplaycode.util.Vector3;

@TeleOp(name = "Motor Test", group = "Main")
public class MotorTest extends OpMode {
    Robot robot;

    @Override
    public void init() {
        final HardwareCollection hwCollection = new HardwareCollection(hardwareMap);
        telemetry.addData("Status", "Initialized");
        System.out.println("init");

        robot = new Robot(new Vector3(), this);
        robot.startHardwareLoop();
    }

    @Override
    public void loop() {
        telemetry.addData("Susan Motor 1", robot.hardwareCollection.susanMotor1.getEncoderPosition());
        telemetry.addData("Susan Motor 2", robot.hardwareCollection.susanMotor2.getEncoderPosition());
        telemetry.addData("Slide Motor 1", robot.hardwareCollection.slidesMotor1.getEncoderPosition());
        telemetry.addData("Slide Motor 2", robot.hardwareCollection.slidesMotor2.getEncoderPosition());
    }
}
