package org.firstinspires.ftc.teamcode.Teleop.Subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;
@Config

public class Hood {


    // range is 0 -> 0.73
    public Servo hoodServo;

    double currentPos;
    public static double outtakePos;

    public static double recoilStep =0.01;

    public int ballsShot;

    public boolean hoodComp;

    public final int maxBalls = 3;

    public final double velThreshold=200;//RPM
    public final long debounceTimeMS=200;

    public double lastVel,lastShotTime;


    //constructor
    public Hood(OpMode opMode){
        hoodServo = opMode.hardwareMap.get(Servo.class, "hood");
        hoodServo.setDirection(Servo.Direction.REVERSE);
    }
    // subsystem specific methods

    public void incrementHood(){
        if(outtakePos<=0.7)outtakePos+=0.05;
        hoodServo.setPosition(outtakePos);
    }

    public void decrementHood(){
        if (outtakePos>=0)outtakePos-=0.05;
        hoodServo.setPosition(outtakePos);
    }
    public void goToHood(double pos){
        hoodServo.setPosition(pos);
    }

    public double getCurrentPos(){return hoodServo.getPosition();}

    public double getPos(){
        return outtakePos;
    }

    public double clamp(double val,double maxPos,double minPos){
        return Math.min(maxPos,Math.max(val,minPos));
    }

    public void updateHood(){
        if (Turret.distance>=110 && getCurrentPos()!=0.3) {
            goToHood(0.3);
            hoodComp=false;
        }
        if(Turret.distance<110) {
            goToHood(0.25);
            hoodComp = true;
        }


    }

    public void onShot(){
        ballsShot++;

        if (ballsShot<=maxBalls){
            outtakePos-= recoilStep;
            hoodServo.setPosition(outtakePos);
        }

        if(ballsShot>=maxBalls){
            reset();
        }

    }

    public void reset(){
        ballsShot=0;
        outtakePos = hoodComp ? 0.3 : 0.25;
    }

    public boolean detectBalls(double v){
        long now = System.currentTimeMillis();
        double dt = now - lastShotTime;
        double dv = v - lastVel;
        lastVel=v;

        return dv >= velThreshold && dt >= debounceTimeMS;


    }


}
