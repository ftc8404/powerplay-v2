package org.quixilver8404.powerplay.control;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.quixilver8404.powerplay.hardware.misc.Clock;
import org.quixilver8404.powerplay.hardware.motor.EncoderMotor;
import org.quixilver8404.powerplay.hardware.motor.EncoderlessMotor;
import org.quixilver8404.powerplay.hardware.sensor.DeltaEncoder;
import org.quixilver8404.powerplay.hardware.sensor.BNO055IMU;
//import org.quixilver8404.powerplay.hardware.sensor.IMU;
import org.quixilver8404.powerplay.hardware.servo.PositionServo;
import org.quixilver8404.powerplay.hardware.sensor.MaxbotixMB1242;
import org.quixilver8404.powerplay.util.Tunable;

/**
 * Stores every hardware device attached to the robot
 */
public class HardwareCollection {

    // clock
    public final Clock clock;

    // control and expansion hub
    public LynxModule controlHub;
    public LynxModule expansionHub;

    // drive motors
    public final EncoderlessMotor driveMotorFL;
    public final EncoderlessMotor driveMotorFR;
    public final EncoderlessMotor driveMotorBL;
    public final EncoderlessMotor driveMotorBR;

    // drive motor direction tunables
    @Tunable
    public static final DcMotorSimple.Direction DRIVE_MOTOR_FL_DIRECTION = DcMotorSimple.Direction.FORWARD;
    @Tunable
    public static final DcMotorSimple.Direction DRIVE_MOTOR_FR_DIRECTION = DcMotorSimple.Direction.REVERSE;
    @Tunable
    public static final DcMotorSimple.Direction DRIVE_MOTOR_BL_DIRECTION = DcMotorSimple.Direction.FORWARD;
    @Tunable
    public static final DcMotorSimple.Direction DRIVE_MOTOR_BR_DIRECTION = DcMotorSimple.Direction.REVERSE;

    // drive encoders
    public final DeltaEncoder driveEncoderLeft;
    public final DeltaEncoder driveEncoderRight;
    public final DeltaEncoder driveEncoderCenter;

    // drive encoder tunables
    @Tunable
    public static final DcMotorSimple.Direction DRIVE_ENCODER_LEFT_DIRECTION = DcMotorSimple.Direction.REVERSE;
    @Tunable
    public static final DcMotorSimple.Direction DRIVE_ENCODER_RIGHT_DIRECTION = DcMotorSimple.Direction.REVERSE;
    @Tunable
    public static final DcMotorSimple.Direction DRIVE_ENCODER_CENTER_DIRECTION = DcMotorSimple.Direction.REVERSE;

    //lazy susan motors
    public final EncoderMotor susanMotor1;
    public final EncoderlessMotor susanMotor2;

    public static final DcMotorSimple.Direction SUSAN_MOTOR_1_DIRECTION = DcMotorSimple.Direction.FORWARD;
    public static final DcMotorSimple.Direction SUSAN_MOTOR_2_DIRECTION = DcMotorSimple.Direction.FORWARD;

    public final EncoderMotor slidesMotor1;
    public final EncoderlessMotor slidesMotor2;

    public static final DcMotorSimple.Direction SLIDES_MOTOR_1_DIRECTION = DcMotorSimple.Direction.REVERSE;
    public static final DcMotorSimple.Direction SLIDES_MOTOR_2_DIRECTION = DcMotorSimple.Direction.FORWARD;

    public final PositionServo gearServo;
    public static final Servo.Direction GEAR_SERVO_DIRECTION = Servo.Direction.FORWARD;

//    public final OpenCvCamera camera;

    public final MaxbotixMB1242 ultraSonic1;
    public final MaxbotixMB1242 ultraSonic2;
    public final MaxbotixMB1242 ultraSonic3;

//    public final IMU controlIMU;

    /**
     * May block slightly as all hardware is initialized, servos may snap to their
     * default positions
     */
    public HardwareCollection(HardwareMap hwMap) {
        // utilities
        clock = new Clock("globalClock");

        // for bulk reading
        controlHub = hwMap.get(LynxModule.class, "Control Hub");
//        controlHub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        refreshControlHubBulkData();
        expansionHub = hwMap.get(LynxModule.class, "Expansion Hub");
//        expansionHub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        refreshExpansionHubBulkData();

        // drive motors
        driveMotorFL = new EncoderlessMotor("driveMotorFL", DRIVE_MOTOR_FL_DIRECTION, hwMap);
        driveMotorFR = new EncoderlessMotor("driveMotorFR", DRIVE_MOTOR_FR_DIRECTION, hwMap);
        driveMotorBL = new EncoderlessMotor("driveMotorBL", DRIVE_MOTOR_BL_DIRECTION, hwMap);
        driveMotorBR = new EncoderlessMotor("driveMotorBR", DRIVE_MOTOR_BR_DIRECTION, hwMap);

        // odometry encoders
        driveEncoderLeft = new DeltaEncoder(driveMotorBR, DRIVE_ENCODER_LEFT_DIRECTION);
        driveEncoderRight = new DeltaEncoder(driveMotorBL, DRIVE_ENCODER_RIGHT_DIRECTION);
        driveEncoderCenter = new DeltaEncoder(driveMotorFL, DRIVE_ENCODER_CENTER_DIRECTION);

        susanMotor1 = new EncoderMotor("susanMotor1", SUSAN_MOTOR_1_DIRECTION, hwMap);
        susanMotor2 = new EncoderlessMotor("susanMotor2", SUSAN_MOTOR_2_DIRECTION, hwMap);

        slidesMotor1 = new EncoderMotor("slidesMotor1", SLIDES_MOTOR_1_DIRECTION, hwMap);
        slidesMotor2 = new EncoderlessMotor("slidesMotor2", SLIDES_MOTOR_2_DIRECTION, hwMap);

        gearServo = new PositionServo("gearServo", GEAR_SERVO_DIRECTION, hwMap);
        ultraSonic1 = hwMap.get(MaxbotixMB1242.class, "ultraSonic1");
        ultraSonic2 = hwMap.get(MaxbotixMB1242.class, "ultraSonic2");
        ultraSonic3 = hwMap.get(MaxbotixMB1242.class, "ultraSonic3");

//        controlIMU =  new IMU("controlIMU", hwMap);

        final int cameraMonitorViewId = hwMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hwMap.appContext.getPackageName());
        final WebcamName webcamName = hwMap.get(WebcamName.class, "webcam");
//        camera = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);
//        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
//            @Override
//            public void onOpened() {
//                System.out.println("OPENED-CAMERA");
//                camera.setViewportRenderer(OpenCvCamera.ViewportRenderer.GPU_ACCELERATED);
//                camera.startStreaming(800, 448, OpenCvCameraRotation.UPRIGHT);
//                System.out.println("STARTED-STREAMING");
//            }
//
//            @Override
//            public void onError(int errorCode) {
//
//            }
//        });
    }

    /**
     * Reads bulk data from expansion hub 1
     */
    public void refreshControlHubBulkData() {
//        controlHub.getBulkData();
    }

    /**
     * Reads bulk data from expansion hub 2
     */
    public void refreshExpansionHubBulkData() {
//        expansionHub.getBulkData();
    }
}
