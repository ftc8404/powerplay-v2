package org.quixilver8404.powerplaycode.opmodes;


import static java.lang.Thread.sleep;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.quixilver8404.powerplaycode.control.base.HardwareCollection;
import org.quixilver8404.powerplaycode.control.base.Robot;
import org.quixilver8404.powerplaycode.hardware.servos.BaseServo;


@TeleOp(name = "Servo Test", group = "Main")
public class ServoTest extends OpMode {

    public static Robot robot;
    int servoIndex = 0;
    double servoPos = 0.5;
    boolean servoPowered = false;
    boolean dpadRightReleased = true;
    boolean dpadLeftReleased = true;
    BaseServo[] servos = {};

    @Override
    public void init() {
        final HardwareCollection hwCollection = new HardwareCollection(hardwareMap);

        servos = new BaseServo[] {hwCollection.gearServo};
        servoIndex = 0;
        servoPos = 0.5;
        servoPowered = false;
        dpadRightReleased = true;
        dpadLeftReleased = true;
    }

    @Override
    public void loop() {
        if (gamepad1.dpad_up) {
            servoPos += 0.01;
        } else if (gamepad1.dpad_down) {
            servoPos -= 0.01;
        }

        if (servoPos < 0) {
            servoPos = 0;
        } else if (servoPos > 1) {
            servoPos = 1;
        }

        if (gamepad1.left_bumper || gamepad1.right_bumper) {
            servos[servoIndex].setPosition(servoPos);
            servoPowered = true;
        }

        boolean reset = false;
        int prevServoIndex = servoIndex;
        if (gamepad1.dpad_right) {
            if (dpadRightReleased) {
                dpadRightReleased = false;
                servoIndex++;
                if (servoIndex >= servos.length) {
                    servoIndex = 0;
                }
                reset = true;
            }
        } else {
            dpadRightReleased = true;
        }

        if (gamepad1.dpad_left) {
            if (dpadLeftReleased) {
                dpadLeftReleased = false;
                servoIndex--;
                if (servoIndex < 0) {
                    servoIndex = servos.length - 1;
                }
                reset = true;
            }
        } else {
            dpadLeftReleased = true;
        }

        if (gamepad1.x) {
            reset = true;
        }

        if (reset) {
            servoPowered = false;
            servoPos = 0.5;
            servos[prevServoIndex].disable();
        }

        telemetry.addData("servo name", servos[servoIndex].getDeviceName());
        telemetry.addData("servo index (dpad left/right)", servoIndex);
        telemetry.addData("servo position (dpad up/down)", "%.2f", servoPos);
        telemetry.addData("servo actual position", servos[servoIndex].getPosition());
        telemetry.addData("servo powered (x)", servoPowered);
        telemetry.update();
    }
}
