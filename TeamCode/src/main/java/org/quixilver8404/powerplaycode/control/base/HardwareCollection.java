package org.quixilver8404.powerplaycode.control.base;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.lynx.LynxI2cDeviceSynchV2;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.lynx.commands.standard.LynxSetModuleLEDColorCommand;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchDevice;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.UltrasonicSensor;

//import org.openftc.easyopencv.OpenCvCamera;
//import org.openftc.easyopencv.OpenCvCameraFactory;
//import org.openftc.easyopencv.OpenCvCameraRotation;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.quixilver8404.powerplaycode.hardware.misc.Clock;
import org.quixilver8404.powerplaycode.hardware.misc.IMU;
import org.quixilver8404.powerplaycode.hardware.motors.BaseMotor;
import org.quixilver8404.powerplaycode.hardware.motors.EncoderMotor;
import org.quixilver8404.powerplaycode.hardware.motors.EncoderPositionMotor;
import org.quixilver8404.powerplaycode.hardware.motors.EncoderlessMotor;
import org.quixilver8404.powerplaycode.hardware.sensors.Encoder;
import org.quixilver8404.powerplaycode.hardware.servos.BaseServo;
import org.quixilver8404.powerplaycode.hardware.sensors.UltrasonicI2cRangeSensor;

public class HardwareCollection {

    // clock
    public final Clock clock;

    // rev hubs
    public final LynxModule revHub1;
    public final LynxModule revHub2;

    public LynxSetModuleLEDColorCommand colorCommandControl;
    public LynxSetModuleLEDColorCommand colorCommandExpansion;

    // drive motors
    public final EncoderlessMotor driveMotorFR;
    public final EncoderlessMotor driveMotorFL;
    public final EncoderlessMotor driveMotorBL;
    public final EncoderlessMotor driveMotorBR;

    //lazy susan motors
    public final EncoderMotor susanMotor1;
    public final EncoderMotor susanMotor2;

    public final EncoderMotor slidesMotor1;
    public final EncoderMotor slidesMotor2;

    // drive motor direction constants
    public static final DcMotorSimple.Direction DRIVE_MOTOR_FR_DIRECTION = DcMotorSimple.Direction.REVERSE;
    public static final DcMotorSimple.Direction DRIVE_MOTOR_FL_DIRECTION = DcMotorSimple.Direction.FORWARD;
    public static final DcMotorSimple.Direction DRIVE_MOTOR_BL_DIRECTION = DcMotorSimple.Direction.FORWARD;
    public static final DcMotorSimple.Direction DRIVE_MOTOR_BR_DIRECTION = DcMotorSimple.Direction.REVERSE;

    // ultrasonic sensors
    public final UltrasonicI2cRangeSensor ultraSonic1;
    public I2cDevice i2cDevice1;

    // odometry encoders also called odometers
    public final Encoder odometryEncoder1;
    public final Encoder odometryEncoder2;
    public final Encoder odometryEncoder3;

    // odometer encoder constants
    public static final DcMotorSimple.Direction ODOMETRY_ENCODER_1_DIRECTION = DcMotorSimple.Direction.REVERSE;
    public static final DcMotorSimple.Direction ODOMETRY_ENCODER_2_DIRECTION = DcMotorSimple.Direction.FORWARD;
    public static final DcMotorSimple.Direction ODOMETRY_ENCODER_3_DIRECTION = DcMotorSimple.Direction.FORWARD;

    public final IMU imu;

    public static final Servo.Direction OUTPUT_SERVO1_DIRECTION = Servo.Direction.FORWARD;
    public static final Servo.Direction OUTPUT_SERVO2_DIRECTION = Servo.Direction.REVERSE;

    public final BaseServo gearServo;
    public static final Servo.Direction GEAR_SERVO_DIRECTION = Servo.Direction.FORWARD;
    public final OpenCvCamera camera;

//    public final ColorSensor colorSensor;

    /**
     * Initializes the utilities, rev hubs and drive motors
     * @param hwMap - part of linear op mode and has a reference to all the hardware
     */
    public HardwareCollection(final HardwareMap hwMap) {
        // utilities
        clock = new Clock("globalClock");
        imu = new IMU(hwMap, "imu", BNO055IMU.AngleUnit.RADIANS, BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC);

        // rev hubs for bulk reading (at the end so bulk caching doesn't mess up other stuff
        hwMap.logDevices();
        revHub1 = hwMap.get(LynxModule.class, "Control Hub");
        revHub1.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        revHub1.getBulkData();

        revHub2 = hwMap.get(LynxModule.class, "Expansion Hub");
        revHub2.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        revHub2.getBulkData();

        // drive motors
        driveMotorFL = new EncoderlessMotor("driveMotorFL", DRIVE_MOTOR_FR_DIRECTION, hwMap);
        driveMotorFL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        driveMotorFR = new EncoderlessMotor("driveMotorFR", DRIVE_MOTOR_FL_DIRECTION, hwMap);
        driveMotorFR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        driveMotorBR = new EncoderlessMotor("driveMotorBR", DRIVE_MOTOR_BL_DIRECTION, hwMap);
        driveMotorBR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        driveMotorBL = new EncoderlessMotor("driveMotorBL", DRIVE_MOTOR_BR_DIRECTION, hwMap);
        driveMotorBL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

//        i2cDevice1 = hwMap.i2cDevice.get("ultraSonic");
//        ultraSonic1 = hwMap.get(UltrasonicI2cRangeSensor::class.java,"ultraSonic");
        ultraSonic1 = new UltrasonicI2cRangeSensor((I2cDeviceSynch) hwMap.get(I2cDeviceSynchDevice.class,"ultraSonic").getDeviceClient());

//        ultraSonic2 = hwMap.ultrasonicSensor.get("ultraSonic2");

//        intakeMotor = new BaseMotor("intakeMotor", INTAKE_MOTOR_DIRECTION, hwMap);
//        intakeMotor.setMaxPower(0.8);

//        carouselMotor = new BaseMotor("carouselMotor", CAROUSEL_MOTOR_DIRECTION, DcMotor.ZeroPowerBehavior.BRAKE, hwMap);

//        final BaseMotor chainMotorB = new BaseMotor("chainMotorB", DcMotorSimple.Direction.FORWARD, hwMap);
//        final BaseMotor chainMotorF = new BaseMotor("chainMotorF", DcMotorSimple.Direction.FORWARD, hwMap);
//        final BaseMotor fakeMotor3 = new BaseMotor("fakeMotor", DcMotorSimple.Direction.FORWARD, hwMap);

//        final BaseMotor fakeMotor1 = new BaseMotor("fakeMotor1", DcMotorSimple.Direction.FORWARD, hwMap);
//        final BaseMotor fakeMotor2 = new BaseMotor("fakeMotor2", DcMotorSimple.Direction.FORWARD, hwMap);
//        final BaseMotor fakeMotor3 = new BaseMotor("fakeMotor3", DcMotorSimple.Direction.FORWARD, hwMap);

        susanMotor1 = new EncoderMotor("susanMotor1", DcMotorSimple.Direction.FORWARD, DcMotor.ZeroPowerBehavior.BRAKE, hwMap, revHub1);
        susanMotor2 = new EncoderMotor("susanMotor2", DcMotorSimple.Direction.FORWARD, DcMotor.ZeroPowerBehavior.BRAKE, hwMap, revHub1);

        slidesMotor1 = new EncoderMotor("slidesMotor1", DcMotorSimple.Direction.REVERSE, DcMotor.ZeroPowerBehavior.BRAKE, hwMap, revHub1);
        slidesMotor2 = new EncoderMotor("slidesMotor2", DcMotorSimple.Direction.FORWARD, DcMotor.ZeroPowerBehavior.BRAKE, hwMap, revHub1);


//        final Encoder liftMotorEncoder = new Encoder(intakeMotor, DcMotorSimple.Direction.FORWARD, revHub1);

        gearServo = new BaseServo("gearServo", Servo.Direction.FORWARD, hwMap);

        // odometry encoders
        // remember to update hardware connections
        odometryEncoder1 = new Encoder(driveMotorBR, ODOMETRY_ENCODER_1_DIRECTION, revHub1);
        odometryEncoder2 = new Encoder(driveMotorBL, ODOMETRY_ENCODER_2_DIRECTION, revHub1);
        odometryEncoder3 = new Encoder(driveMotorFL, ODOMETRY_ENCODER_3_DIRECTION, revHub1);

//        System.out.println("Started-t265: " + slamra.isStarted());

        final int cameraMonitorViewId = hwMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hwMap.appContext.getPackageName());
        final WebcamName webcamName = hwMap.get(WebcamName.class, "webcam");
        camera = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                System.out.println("OPENED-CAMERA");
                camera.setViewportRenderer(OpenCvCamera.ViewportRenderer.GPU_ACCELERATED);
                camera.startStreaming(800, 448, OpenCvCameraRotation.UPRIGHT);
                System.out.println("STARTED-STREAMING");
            }

            @Override
            public void onError(int errorCode) {

            }
        });


        odometryEncoder1.reset();
        odometryEncoder2.reset();
        odometryEncoder3.reset();

//        colorSensor = hwMap.get(ColorSensor.class, "colorSensor");
//        colorSensor.enableLed(false);
    }

    public void refreshBulkData() {
        revHub1.getBulkData();
        revHub2.getBulkData();
        clock.update();
        imu.update();
    }
}
