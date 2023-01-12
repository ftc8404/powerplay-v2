package org.quixilver8404.skystone.control;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.quixilver8404.skystone.util.measurement.Pose2D;

public class TeleOpRobot extends BaseRobot {

    public TeleOpRobot(Pose2D startPose,LinearOpMode opMode) {
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
