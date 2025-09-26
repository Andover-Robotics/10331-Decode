package org.firstinspires.ftc.teamcode.Teleop.Subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@Config
public class Intake {
    public DcMotorEx intakeMotor;
    public static double power = 0.0;
    public Intake (OpMode opMode){
        intakeMotor.setDirection(DcMotorEx.Direction.FORWARD);
        intakeMotor= opMode.hardwareMap.get(DcMotorEx.class, "intake motor");    }

    public void intake(){
        intakeMotor.setDirection(DcMotorEx.Direction.FORWARD);
        intakeMotor.setPower(power);
    }

    public void reverseIntake(){
        intakeMotor.setDirection(DcMotorEx.Direction.REVERSE);
        intakeMotor.setPower(power);
    }

    public void stopIntake(){
        intakeMotor.setPower(0);
    }

}
