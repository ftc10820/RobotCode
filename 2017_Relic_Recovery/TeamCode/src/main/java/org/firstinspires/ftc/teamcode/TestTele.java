package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by the Falconeers 10820
 */


import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;
import com.sun.tools.javac.util.Context;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

import java.util.Locale;


/**
 * Created by the Falconeers 10820
 */
@TeleOp(name = "Test Tele", group = "Test")
public class TestTele extends OpMode {

//    private DcMotor leftDrive = null;
//
//    private double clawOffset = 0.0;                  // Servo mid position
//    private final double CLAW_SPEED = 0.02;                 // Sets rate to move servo
//
//    private static final double MID_SERVO = 0.5;
//
//    private Servo leftClaw = null;
//    private Servo rightClaw = null;
    //    Telemetry telemetry = null;
    BNO055IMU imu;
    Orientation angles;
    Acceleration gravity;


    // Most robots need the motor on one side to be reversed to drive forward
//        // Reverse the motor that runs backwards when connected directly to the battery
//        leftDrive.setDirection(DcMotor.Direction.FORWARD);
////        rightDrive.setDirection(DcMotor.Direction.REVERSE);
//
//
//    }
//    public void claw(Boolean willOpen) {
//
//        if (willOpen)
//            clawOffset += CLAW_SPEED;
//        else
//            clawOffset -= CLAW_SPEED;
//
//        // Move both servos to new position.  Assume servos are mirror image of each other.
//        clawOffset = Range.clip(clawOffset, -0.5, 0.5);
//        leftClaw.setPosition(MID_SERVO + clawOffset);
//        rightClaw.setPosition(MID_SERVO - clawOffset);
//    }
    //    public void elevatorMove(int isGoingUp){
//        elevatorMotor.setPower(isGoingUp);
//        telemetry.addData("Elevator Motor", elevatorMotor.getCurrentPosition());
//    }
//    public void drive(double leftPower, double rightPower) {
//
//        final double MINIMUMOFFSET = 0.1;
//
//        // Send calculated power to wheels
//        if (MINIMUMOFFSET <= Math.abs(leftPower)) {
//
//
//        }
////        if (MINIMUMOFFSET <= Math.abs(rightPower)) {
////            rightDrive.setPower(rightPower);
////
////        }
//        telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);
//    }

    @Override
    public void init() {
        this.msStuckDetectInit = 10_000;
//        public void initHardware(HardwareMap hardwareMap, Telemetry telemetry){
//        leftDrive = hardwareMap.get(DcMotor.class, "Motor 0");
//        rightClaw = hardwareMap.get(Servo.class, "Servo 0");
//        leftClaw = hardwareMap.get(Servo.class, "Servo 1");

//        leftClaw.setPosition(MID_SERVO);
//        rightClaw.setPosition(MID_SERVO);
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
        imu.startAccelerationIntegration(new Position(), new Velocity(), 100);
        //  this.telemetry = telemetry;
        //}
    }

    @Override
    public void loop() {
//        leftDrive.setPower(gamepad1.right_stick_x);
//        if (gamepad1.a) claw(true);
//        if (gamepad1.y) claw(false);
        telemetry.addAction(new Runnable() {
            @Override
            public void run() {
                // Acquiring the angles is relatively expensive; we don't want
                // to do that in each of the three items that need that info, as that's
                // three times the necessary expense.
                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                gravity = imu.getGravity();
            }
        });

        telemetry.addLine()
                .addData("status", new Func<String>() {
                    @Override
                    public String value() {
                        return imu.getSystemStatus().toShortString();
                    }
                })
                .addData("calib", new Func<String>() {
                    @Override
                    public String value() {
                        return imu.getCalibrationStatus().toString();
                    }
                });

        telemetry.addLine()
                .addData("heading", new Func<String>() {
                    @Override
                    public String value() {
                        return formatAngle(angles.angleUnit, angles.firstAngle);
                    }
                })
                .addData("roll", new Func<String>() {
                    @Override
                    public String value() {
                        return formatAngle(angles.angleUnit, angles.secondAngle);
                    }
                })
                .addData("pitch", new Func<String>() {
                    @Override
                    public String value() {
                        return formatAngle(angles.angleUnit, angles.thirdAngle);
                    }
                });

        telemetry.addLine()
                .addData("grvty", new Func<String>() {
                    @Override
                    public String value() {
                        return gravity.toString();
                    }
                })
                .addData("mag", new Func<String>() {
                    @Override
                    public String value() {
                        return String.format(Locale.getDefault(), "%.3f",
                                Math.sqrt(gravity.xAccel * gravity.xAccel
                                        + gravity.yAccel * gravity.yAccel
                                        + gravity.zAccel * gravity.zAccel));
                    }
                });
    }

    //    }
//
//    ----------------------------------------------------------------------------------------------
//     Formatting
//    ----------------------------------------------------------------------------------------------
//
    String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    String formatDegrees(double degrees) {
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }
}


//
/*

void floodfill(int x, int y){
    if(x < 0 || x >= WIDTH || y < 0 || y >= HEIGHT || bmarr[x][y]) return;
    bmarr[x][y];
    floodfill(x + 1, y);
    floodfill(x - 1, y);
    floodfill(x, y + 1);
    floodfill(x, y - 1);
}

bitmap bm = getBitMap();
boolean[][] bmarr = toArray(bm);
for x from 0 to WIDTH - 1
    floodFill(x, 0);
    floodFill(x, HEIGHT - 1);
for y from 0 to HEIGHT - 1
    floodFill(0, y);
    floodFill(WIDTH - 1, y);

for (x, y) from in [0, WIDTH - 1] * [0, HEIGHT - 1]
    if(!bmarr[x][y]) {
        if(isBlockWeWant(x, y)) pickUpBlock();
        else floodfill(x, y);
    }

*/