package org.firstinspires.ftc.robotcontroller;

/**
 * Created by the Falconeers 10820
 */

public class MecanumDrive extends BasicDriveTrain {
    public void der(double x, double y){
        // x = 1 for left
        // x = -1 for right
        double velocity = 1;
        double pi = Math.PI;
        double piDiv4 = pi/4;
        double theta = Math.atan2(y,x);
        while(theta < 0) theta += Math.PI * 2;
        frontLeft.setPower(-velocity * Math.sin(theta + piDiv4));
        backLeft.setPower(velocity * Math.cos(theta + piDiv4));
        frontRight.setPower(-velocity * Math.cos(theta + piDiv4));
        backRight.setPower(velocity * Math.sin(theta + piDiv4));
    }
    public void derLeft() {
        der(1,0);
    }

    public void derRight() {
        der(-1,0);
    }

}
