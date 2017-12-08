package org.firstinspires.ftc.robotcontroller;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcontroller.MecanumDrive;

/**
 * Created by the Falconeers 10820
 */
@TeleOp(name = "DEMOBOT", group = "demos")
public class DemoBot extends OpMode {

    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;
    private DcMotor spinner;

    @Override
    public void init() {
        frontLeft = hardwareMap.dcMotor.get("motor_frontLeft");
        frontRight = hardwareMap.dcMotor.get("motor_frontRight");
        backLeft = hardwareMap.dcMotor.get("motor_backLeft");
        backRight = hardwareMap.dcMotor.get("motor_backRight");
        spinner = hardwareMap.dcMotor.get("motor_spinner");

    }
    private void drive(short d) {
        frontLeft.setPower(-d);
        backLeft.setPower(-d);
        frontRight.setPower(d);
        backRight.setPower(d);
    }

    @Override
    public void loop() {
        double frontL = 0;
        double backL = 0;
        double frontR = 0;
        double backR = 0;
        double theta;

        double v0 = 0;
        //double pi = 3.1459265359;
        double piDiv4 = Math.PI / 4;
        spinner.setPower(gamepad1.right_trigger);
        if (gamepad1.right_bumper) {
            if (Math.abs(gamepad1.right_stick_x) >= .1 || Math.abs(gamepad1.right_stick_y) >= .1) {
                theta = Math.atan2(-gamepad1.right_stick_y, gamepad1.right_stick_x);
                while (theta < 0) theta += Math.PI * 2;
                double velocity = 1;
                frontL = -velocity * Math.sin(theta + piDiv4) + v0;
                backL = velocity * Math.cos(theta + piDiv4) + v0;
                frontR = -velocity * Math.cos(theta + piDiv4) + v0;
                backR = velocity * Math.sin(theta + piDiv4) + v0;
            }
        } else {
            double mod = 1;
            frontL = gamepad1.right_stick_y * mod;
            backL = gamepad1.right_stick_y * mod;
            frontR = -gamepad1.left_stick_y * mod;
            backR = -gamepad1.left_stick_y * mod;
        }
        double largestVal = Math.max(Math.max(Math.abs(frontL), Math.abs(backL)), (Math.max(Math.abs(frontR), Math.abs(backR))));
        if (largestVal > 1) {
            frontL /= largestVal;
            backL /= largestVal;
            frontR /= largestVal;
            backR /= largestVal;
        }

        frontLeft.setPower(frontL);
        backLeft.setPower(backL);
        frontRight.setPower(frontR);
        backRight.setPower(backR);
    }
}
