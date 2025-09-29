package org.firstinspires.ftc.teamcode.Teleop.Subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.motors.CRServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import com.qualcomm.robotcore.hardware.DigitalChannel;


@Config
public class Intake {
    public DcMotorEx intakeMotor;
    public CRServo toilet3;
    public static double power = 0.0;
    public static double betterPower = 0.0;

    DigitalChannel breakBeam;


    public Intake (OpMode opMode){
        intakeMotor.setDirection(DcMotorEx.Direction.FORWARD);
        intakeMotor = opMode.hardwareMap.get(DcMotorEx.class, "intake motor");
        toilet3 = opMode.hardwareMap.get(CRServo.class,"toilet roll 3");
        breakBeam = opMode.hardwareMap.get(DigitalChannel.class, "BreakBeam");
    }

    public void intake_without_sense(){
        intakeMotor.setDirection(DcMotorEx.Direction.FORWARD);
        intakeMotor.setPower(power);
        toilet3.set(betterPower);
    }

    public void reverseIntake(){
        intakeMotor.setDirection(DcMotorEx.Direction.REVERSE);
        intakeMotor.setPower(power);
        toilet3.set(-betterPower);
    }

    public void stopIntake(){
        intakeMotor.setPower(0);
        toilet3.set(0);
    }


    public boolean getSensorState(){
        return breakBeam.getState();
        // true= unbroken
        // false = broken
    }

}
