package org.quixilver8404.powerplay.hardware.sensor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.quixilver8404.powerplay.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.quixilver8404.powerplay.control.BaseRobot;
import org.quixilver8404.powerplay.util.measurement.Angle;

public class IMU extends HardwareDevice {
    com.qualcomm.robotcore.hardware.IMU imu;
    public IMU(String deviceName, HardwareMap hwMap){
        super(deviceName);
        imu = hwMap.get(com.qualcomm.robotcore.hardware.IMU.class, deviceName);
    }
    public double getYaw(){
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
    }
    public double getYawDeg(){
        return (imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES)+360)%360;
    }
    public double getPitch(){
        return imu.getRobotYawPitchRollAngles().getPitch(AngleUnit.RADIANS);
    }
    public double getRoll(){
        return imu.getRobotYawPitchRollAngles().getRoll(AngleUnit.RADIANS);
    }
    public double getPitchDeg(){
        return imu.getRobotYawPitchRollAngles().getPitch(AngleUnit.DEGREES);
    }
    public double getRollDeg(){
        return imu.getRobotYawPitchRollAngles().getRoll(AngleUnit.DEGREES);
    }
    public void resetIMU(){
        imu.resetYaw();
    }
}
