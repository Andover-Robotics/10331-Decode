package org.firstinspires.ftc.teamcode.Teleop.Subsystems;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;

public class Shooter {
    public Servo hoodServo;
    public MotorEx shooter;
    public static double shooterSpeed = 0.0;
    private boolean shoot=false;

    //constructor
    public Shooter(OpMode opMode){
        hoodServo = opMode.hardwareMap.get(Servo.class, "hood");
        shooter = new MotorEx(opMode.hardwareMap, "shooter", Motor.GoBILDA.RPM_1620); // idk why no 6000 but ok

    }

    //may need to change this class based on final outtake
    // subsystem specific methods
    public void shoot(){
        if (shoot){
            shooter.stopMotor();
            shoot=false;
        }
        else {
            shooter.setVelocity(shooterSpeed);
            shoot=true;

        }
    }
}
