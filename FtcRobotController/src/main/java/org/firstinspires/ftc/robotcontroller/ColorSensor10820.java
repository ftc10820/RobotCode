package org.firstinspires.ftc.robotcontroller;

import android.hardware.Sensor;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchImpl;

/**
 * Created by the Falconeers 10820
 */

public class ColorSensor10820{
    public I2cDevice colorSensor;
    public byte[] colorSensorCache;
    public I2cDeviceSynch colorSensorReader;
    public int i2cIdentifier;

    public void setValues(I2cDevice colorSenor, int i2cIdentifier){
        this.colorSensor = colorSenor;
        this.i2cIdentifier = i2cIdentifier;
    }

    public void init() {
        colorSensorReader = new I2cDeviceSynchImpl(colorSensor, I2cAddr.create8bit(i2cIdentifier),false);
        colorSensorReader.engage();
    }

    public void setPassive(){
        colorSensorReader.write8(3, 1);
    }
    public void setActive(){
        colorSensorReader.write8(3, 0);
    }

    public int getColorNum(){
        return colorSensorCache[0x04];
    }

    public int getRedVal() {
        return colorSensorCache[0x05];
    }

    public int getGreenVal() {
        return colorSensorCache[0x06];
    }

    public int getBlueVal() {
        return colorSensorCache[0x07];
    }

    public int getWhiteVal() {
        return colorSensorCache[0x08];
    }



}
