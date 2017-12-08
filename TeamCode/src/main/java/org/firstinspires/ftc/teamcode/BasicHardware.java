package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;
import com.sun.tools.javac.util.Context;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;


/**
 * Created by the Falconeers 10820
 */

public class BasicHardware {

    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;
    private DcMotor elevatorMotor = null;

    private double clawOffset = 0.0;                  // Servo mid position
    private final double CLAW_SPEED = 0.02;                 // Sets rate to move servo

    private static final double MID_SERVO = 0.5;
    // The IMU sensor object
    private BNO055IMU imu;

    // State used for updating telemetry
    private Orientation angles;
    private Acceleration gravity;
    private Servo leftClaw = null;
    private Servo rightClaw = null;
    Telemetry telemetry = null;
    public void initHardware(HardwareMap hardwareMap, Telemetry telemetry){
        leftDrive = hardwareMap.get(DcMotor.class, "left_drive");
        rightDrive = hardwareMap.get(DcMotor.class, "right_drive");
        elevatorMotor = hardwareMap.get(DcMotor.class, "elevator_motor");
        rightClaw = hardwareMap.get(Servo.class, "right_claw_servo");
        leftClaw = hardwareMap.get(Servo.class, "left_claw_servo");
        this.telemetry = telemetry;

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        leftDrive.setDirection(DcMotor.Direction.FORWARD);
        rightDrive.setDirection(DcMotor.Direction.REVERSE);

        leftClaw.setPosition(MID_SERVO);
        rightClaw.setPosition(MID_SERVO);

        // Set up the parameters with which we will use our IMU. Note that integration
        // algorithm here just reports accelerations to the logcat log; it doesn't actually
        // provide positional information.
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        // Start the logging of measured acceleration
        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);
    }
    public void claw(Boolean willOpen) {

        if (willOpen)
            clawOffset += CLAW_SPEED;
        else
            clawOffset -= CLAW_SPEED;

        // Move both servos to new position.  Assume servos are mirror image of each other.
        clawOffset = Range.clip(clawOffset, -0.5, 0.5);
        leftClaw.setPosition(MID_SERVO + clawOffset);
        rightClaw.setPosition(MID_SERVO - clawOffset);
    }

    public void park(){
        // NEVER USE!!!!!
        // PLZ DON'T
        leftDrive.setPower(1);
        rightDrive.setPower(1);
        while(gravity.zAccel <= 9.75);
        leftDrive.setPower(0);
        rightDrive.setPower(0);
    }
    public void elevatorMove(int isGoingUp){
        elevatorMotor.setPower(isGoingUp);
        telemetry.addData("Elevator Motor", elevatorMotor.getCurrentPosition());
    }
    public void drive(double leftPower, double rightPower) {

        final double MINIMUMOFFSET = 0.1;

        // Send calculated power to wheels
            leftDrive.setPower(leftPower);
            rightDrive.setPower(rightPower);

        telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);
    }
}
