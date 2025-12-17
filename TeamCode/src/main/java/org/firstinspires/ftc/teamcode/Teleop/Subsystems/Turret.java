package org.firstinspires.ftc.teamcode.Teleop.Subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.arcrobotics.ftclib.controller.PIDController;


@Config


public class Turret {
    public MotorEx turret;
    public static double p=0,i=0,d=0;
    public double setPoint =0;
    private final PIDController controller;

    public boolean isManual = false;

    public static double manualPower;




    public Turret (OpMode opMode){
        turret = opMode.hardwareMap.get(MotorEx.class,"turret");
        turret.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        turret.setRunMode(Motor.RunMode.RawPower);
        controller = new PIDController(p,i,d);

    }

    // takes in ticks
    public void runTo(int t){
        setPoint = t;
    }




    public void periodic(){
        if (!isManual) {
            controller.setSetPoint(setPoint);
            controller.calculate(turret.getCurrentPosition());
        }


    }
}
