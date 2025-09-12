package org.firstinspires.ftc.teamcode.Teleop.Subsystems;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

public class Intake {
    public MotorEx intake;
    public static double speed = 0.0;
    public boolean intaking=false; // controls direction of intake

    // constructor
    public Intake (OpMode opMode){
        intake = new MotorEx(opMode.hardwareMap, "intake", Motor.GoBILDA.RPM_1150);
    }

    // I don't think we need any more here lowkey
    public void run(){
        if (intaking) {
            intake.stopMotor();
            intaking = false;
        }
        else{
            intake.setVelocity(speed);
            intaking = true;
        }

    }


}
