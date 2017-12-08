package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "Final Tele", group = "Period8")
public class FalconeersTele extends OpMode {
    // Declare OpMode members.
    private BasicHardware hw = new BasicHardware();
    private ElapsedTime runtime = new ElapsedTime();

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        hw.initHardware(hardwareMap,telemetry);

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


    private void drive() {

        double leftPower = gamepad1.left_stick_y;
        double rightPower = gamepad1.right_stick_y;
        final double MINIMUMOFFSET = 0.1;

        // Send calculated power to wheels
        if (MINIMUMOFFSET <= Math.abs(leftPower)|| MINIMUMOFFSET <= Math.abs(rightPower)) {
            hw.drive(Math.pow(leftPower, 3),Math.pow(rightPower, 3));

        }

        telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);
    }


    int elevatorInt;
    @Override
    public void loop() {
        // Setup a variable for each drive wheel to save power level for telemetry

        drive();
        if (gamepad2.x)
            hw.claw(false);
        else if (gamepad2.b)
            hw.claw(true);
        if(gamepad2.right_bumper)
            elevatorInt = 1;
        else if(gamepad2.left_bumper)
            elevatorInt = -1;
        else
            elevatorInt = 0;
        hw.elevatorMove(elevatorInt);
        telemetry.addData("Status", "Run Time: " + runtime.toString());
//        WORK OF THE CODE DEVEL, DO NOT TOUCH!!!!!!!!
//        JUST DON'T
//        if (gamepad2.a)
//            hw.park();

    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

}
