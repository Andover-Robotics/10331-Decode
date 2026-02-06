package org.firstinspires.ftc.teamcode.Teleop;

import com.qualcomm.robotcore.hardware.Servo;

public class Nicole {
    double currentPos;
    public Servo claw;

    public double claw (double currentPos) {
        currentPos= claw.getPosition();
        return currentPos;

    }

    public void open{
        while (currentPos < 1){
            currentPos+=.2;
            }
        claw.setPosition(currentPos);
        }

    }

    public void close{
        while (currentPos>1){
            currentPos-=2;
        }
        claw.setPosition(currentPos);
}
