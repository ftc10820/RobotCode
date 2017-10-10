package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="Basic Tele", group="Iterative Opmode")
public class BasicTeleOpMode extends OpMode
{
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;
    private DcMotor elevatorMotor = null;

    private double clawOffset  = 0.0 ;                  // Servo mid position
    private final double CLAW_SPEED  = 0.02 ;                 // sets rate to move servo

    private static final double MID_SERVO = 0.5 ;

    private Servo leftClaw = null;
    private Servo rightClaw = null;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        leftDrive  = hardwareMap.get(DcMotor.class, "left_drive");
        rightDrive = hardwareMap.get(DcMotor.class, "right_drive");
        elevatorMotor = hardwareMap.get(DcMotor.class, "elevator_motor");
        rightClaw = hardwareMap.get(Servo.class, "right_claw_servo");
        leftClaw = hardwareMap.get(Servo.class, "left_claw_servo");


        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        leftDrive.setDirection(DcMotor.Direction.FORWARD);
        rightDrive.setDirection(DcMotor.Direction.REVERSE);

        leftClaw.setPosition(MID_SERVO);
        rightClaw.setPosition(MID_SERVO);

        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        runtime.reset();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */

    private void elevator(){
        if (gamepad2.right_bumper) {
            elevatorMotor.setPower(1);
        }

        else if (gamepad2.left_bumper) {
            elevatorMotor.setPower(-1);
        }

        else {
            elevatorMotor.setPower(0);
        }

        telemetry.addData("Elevator Motor", elevatorMotor.getCurrentPosition());
    }
    private void drive(){

        double leftPower  = -gamepad1.left_stick_y ;
        double rightPower = -gamepad1.right_stick_y ;
        final double MINIMUMOFFSET = 0.1;

        // Send calculated power to wheels
        if(MINIMUMOFFSET <= Math.abs(leftPower)) {
            leftDrive.setPower(Math.pow(leftPower,3));

        }
        if(MINIMUMOFFSET <= Math.abs(rightPower)) {
            rightDrive.setPower(Math.pow(rightPower,3));

        }
        telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);
    }
    private void claw() {

        if (gamepad2.a)
            clawOffset += CLAW_SPEED;
        else if (gamepad2.x)
            clawOffset -= CLAW_SPEED;

        // Move both servos to new position.  Assume servos are mirror image of each other.
        clawOffset = Range.clip(clawOffset, -0.5, 0.5);
        leftClaw.setPosition(MID_SERVO + clawOffset);
        rightClaw.setPosition(MID_SERVO - clawOffset);
    }
    @Override
    public void loop() {
        // Setup a variable for each drive wheel to save power level for telemetry

        drive();
        claw();
        elevator();
        telemetry.addData("Status", "Run Time: " + runtime.toString());


    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

}
