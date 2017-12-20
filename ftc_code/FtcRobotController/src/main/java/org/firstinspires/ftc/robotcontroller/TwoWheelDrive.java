package org.firstinspires.ftc.robotcontroller;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.GyroSensor;

/**
 * Created by the Falconeers 10820
 */

public class TwoWheelDrive {

    public double diameterOfWheel;
    public double circOfWheel = diameterOfWheel * Math.PI;
    public DcMotor frontWheels;
    public DcMotor backWheels;
    public GyroSensor gyroSensor;
    public double RPM;


    public final short FORWARDS = 1;
    public final short BACKWARDS = -1;
    public final short BRAKE = 0;

    public void setValues(GyroSensor gyroSensor, double RPM, double diameterOfWheel,
                           DcMotor frontWheels, DcMotor backWheels) {
        this.diameterOfWheel = diameterOfWheel;
        this.frontWheels = frontWheels;
        this.backWheels = backWheels;
        this.gyroSensor = gyroSensor;
        this.RPM = RPM;
    }
    public void move(double d) {
        // d is measured in inches
        if (d == 0) return;
        //                                        sec to milli  rpm   per sec
        long time = (long) (Math.abs(d / (circOfWheel)) * 1000 /(RPM / 60));
        if (d > 0) drive(FORWARDS);
        else drive(BACKWARDS);
        waitTime(time);
        drive(BRAKE);
    }




    public void turn(double deg) {
        //left
        if (deg == 0) {
            return;
        }
        while (deg < 0) deg += 360;
        deg = (int) deg % 360;
        if (deg == 0) return;
        if (deg > 180) deg -= 360;

        final int HEADING = gyroSensor.getHeading();
        int target = HEADING + (int) deg;

        while (target < 0) target += 360;
        target = target % 360;

        if (deg < 0) allWheels((short) -1);
        else allWheels((short) 1);
        while(true){
            int heading = gyroSensor.getHeading();
            if (deg < 0){
                if (heading <= target && (heading > HEADING || HEADING > target)) break;
                else if(heading > HEADING && heading > target) break;
            } else {
                if (heading >= target && (heading < HEADING || HEADING < target)) break;
                else if(heading < HEADING && heading < target) break;
            }

        }
        drive(BRAKE);
    }
    public void allWheels(short s) {
        frontWheels.setPower(s);
        backWheels.setPower(s);
    }
    public void drive(short s) {
        frontWheels.setPower(-s);
        backWheels.setPower(-s);
    }
    public void waitTime(long ms) {
        try {
            Thread.sleep(ms);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    }
}

