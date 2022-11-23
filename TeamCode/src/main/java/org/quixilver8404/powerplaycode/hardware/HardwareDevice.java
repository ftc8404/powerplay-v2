package org.quixilver8404.powerplaycode.hardware;

public abstract class HardwareDevice {

    public final String deviceName;

    public HardwareDevice(final String deviceName) {
        this.deviceName = deviceName;
    }

    public String getDeviceName() {
        return deviceName;
    }
}
