package org.quixilver8404.powerplay.hardware.sensor;

import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchDevice;
import com.qualcomm.robotcore.hardware.configuration.I2cSensor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import java.util.concurrent.TimeUnit;

@I2cSensor(name = "Maxbotix MB1242 distance sensor", description = "Distance sensor from Maxbotix", xmlTag = "MaxbotixMB1242")
public class MaxbotixMB1242 extends I2cDeviceSynchDevice<I2cDeviceSynch> {
    public final static I2cAddr ADDRESS_I2C_DEFAULT = I2cAddr.create7bit(0x70);
    private long readInitiatedTime;
    private volatile short lastDistance = 0;

    @Override
    public Manufacturer getManufacturer(){ return Manufacturer.Other; }

    @Override
    protected synchronized boolean doInitialize(){ return true; }

    @Override
    public String getDeviceName(){ return "Maxbotix MB1242 distance sensor"; }

    public MaxbotixMB1242(I2cDeviceSynch deviceClient){
        super(deviceClient, true);
        this.deviceClient.setI2cAddress(ADDRESS_I2C_DEFAULT);
        super.registerArmingStateCallback(false);
        this.deviceClient.engage();
        readInitiatedTime = TimeUnit.NANOSECONDS.toMillis(System.nanoTime());;
    }

    public static short combineHighAndLowBytes(byte highByte, byte lowByte) {
        return (short) ((highByte << 8) | (lowByte & 0xFF));
    }

    public double getDistance(DistanceUnit unit) {
        long currentTime = TimeUnit.NANOSECONDS.toMillis(System.nanoTime());
        if( currentTime > readInitiatedTime + 150) {
            byte[] take_reading = new byte[1];
            take_reading[0] = 0x51;
            this.deviceClient.write(0x70, take_reading);
            readInitiatedTime = currentTime;
            return unit.fromCm(lastDistance);
        } else if (currentTime > readInitiatedTime + 80) {
            byte[] bytes = this.deviceClient.read(0x70, 2);
            lastDistance = combineHighAndLowBytes(bytes[0], bytes[1]);
            return unit.fromCm(lastDistance);
        } else {
            return unit.fromCm(lastDistance);
        }
    }

    public void resetDistance() {
        lastDistance = 0;
    }
}
