package org.quixilver8404.powerplay.control;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.quixilver8404.powerplay.util.Vector3;
import org.quixilver8404.powerplay.util.measurement.Pose2D;

public class AutonRobot extends BaseRobot {

    public AutonRobot(LinearOpMode opMode) {
        super(new Vector3(), true, opMode);

    }
    public AutonRobot(Vector3 startPose, LinearOpMode opMode) {
        super(startPose, true, opMode);

    }

    @Override
    public void update() {
        super.update();
    }

    /**
     * Manually sends commands to the robot right away to stop the drive motors
     */
    public void stopDriveMotors() {
        // TODO fix because this doesn't stop the robot when auton doesn't have enough time
        hwCollection.driveMotorFL.setPower(0);
        hwCollection.driveMotorFR.setPower(0);
        hwCollection.driveMotorBL.setPower(0);
        hwCollection.driveMotorBR.setPower(0);
    }
}
