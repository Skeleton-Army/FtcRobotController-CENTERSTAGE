package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class UltrasonicDistanceSensor implements DistanceSensor {

    private AnalogInput analog;

    public static double MAX_RANGE = 120.0;

    public static double MIN_RANGE = 5;

    public UltrasonicDistanceSensor(AnalogInput analog) {
        this.analog = analog;
    }

    @Override
    public double getDistance(DistanceUnit unit) {
        double inches = (analog.getVoltage() * 312.5)/2.54*1.6857;
        switch (unit) {
            case INCH:
                return inches;
            case METER:
                return inches / 39.37; // converting to meters
        }
        return inches;
    }

    @Override
    public Manufacturer getManufacturer() {
        return Manufacturer.Other;
    }

    @Override
    public String getDeviceName() {
        return null;
    }

    @Override
    public String getConnectionInfo() {
        return null;
    }

    @Override
    public int getVersion() {
        return 0;
    }

    @Override
    public void resetDeviceConfigurationForOpMode() {

    }

    @Override
    public void close() {

    }
}
