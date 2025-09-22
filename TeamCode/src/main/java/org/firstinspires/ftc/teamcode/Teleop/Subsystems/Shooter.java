package org.firstinspires.ftc.teamcode.Teleop.Subsystems;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

public class Shooter {
    public Servo hoodServo;
    public DcMotorEx shooter;
    public static double shooterPower = 0.0;
    double currentPos;
    public static double outtakePos;

    //constructor
    public Shooter(OpMode opMode){
        hoodServo = opMode.hardwareMap.get(Servo.class, "hood");
        shooter = opMode.hardwareMap.get(DcMotorEx.class, "shooter motor");
        currentPos= hoodServo.getPosition();
    }

    //may need to change this class based on final outtake
    // subsystem specific methods
    public void shoot(){
        shooter.setPower(shooterPower);
    }
    public void stop_shoot(){
        shooter.setPower(0);
    }
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
