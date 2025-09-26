package org.firstinspires.ftc.teamcode.Teleop.Subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
@Config

public class Hood {
    public Servo hoodServo;

    double currentPos;
    public static double outtakePos;

    //constructor
    public Hood(OpMode opMode){
        hoodServo = opMode.hardwareMap.get(Servo.class, "hood");
        currentPos= hoodServo.getPosition();
    }
    // subsystem specific methods

    public void incrementHood(){
        currentPos+=0.05;
        if(currentPos>=1){
            currentPos=1;
        }
        hoodServo.setPosition(currentPos);
    }

    public void decrementHood(){
        currentPos-=0.05;
        if(currentPos<=0){
            currentPos=0;
        }
        hoodServo.setPosition(currentPos);
    }
    public void goToHood(){
        hoodServo.setPosition(outtakePos);
    }
}
