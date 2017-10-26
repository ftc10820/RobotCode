package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;
import com.sun.tools.javac.util.Context;

import org.firstinspires.ftc.robotcore.external.Telemetry;


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
    public void elevatorMove(int isGoingUp){
        elevatorMotor.setPower(isGoingUp);
        telemetry.addData("Elevator Motor", elevatorMotor.getCurrentPosition());
    }
    public void drive(double leftPower, double rightPower) {

        final double MINIMUMOFFSET = 0.1;

        // Send calculated power to wheels
        if (MINIMUMOFFSET <= Math.abs(leftPower)) {
            leftDrive.setPower(leftPower);

        }
        if (MINIMUMOFFSET <= Math.abs(rightPower)) {
            rightDrive.setPower(rightPower);

        }
        telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);
    }
}
