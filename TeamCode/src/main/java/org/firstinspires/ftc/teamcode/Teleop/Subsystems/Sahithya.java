package org.firstinspires.ftc.teamcode.Teleop.Subsystems;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;

public class Sahithya {
    public Servo clawServo;

    private double currentPos;

    private static double openPos = .2;

    private static double closedPos = 0.00

    public Sahithya (OpMode opMode){
        clawServo = opMode.hardwareMap.get(Servo.class, "claw");
        currentPos = clawServo.getPosition();
    }
    public void Opens(){
        clawServo.setPosition(openPos);
        currentPos=openPos;
    }

    public void Closes() {
        clawServo.setPosition(closedPos);
        currentPos = closedPos;
    }

    public void incrementClaw(){

        currentPos+=0.05;
        if (currentPos<=1){
            currentPos = 0;
        }
        clawServo.setPosition(currentPos);
    }
    public void decrementClaw(){
        currentPos-=0.05;
        if (currentPos<=1){
            currentPos = 1;
        }
        clawServo.setPosition(currentPos);
    }
}
