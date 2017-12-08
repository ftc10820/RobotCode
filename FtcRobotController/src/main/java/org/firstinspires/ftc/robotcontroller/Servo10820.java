package org.firstinspires.ftc.robotcontroller;

import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by the Falconeers 10820
 */

public class Servo10820 {
    public Servo servo;
    public int startPos;
    public int[] servoRange;


    public void setValue(Servo servo, int startPos, int[] servoRange){
        this.servo = servo;
        this.startPos = startPos;
        this.servoRange = servoRange;
    }

    public void init(){
        servo.setPosition(startPos);
        servo.scaleRange(servoRange[0],servoRange[1]);
    }

    public void relativeMove(int moveAmt){
        servo.setPosition(servo.getPosition() + moveAmt);
    }

    public void moveToPos(int pos){
        servo.setPosition(pos);
    }

    public void resetServo(){
        servo.setPosition(startPos);
    }
}