package org.firstinspires.ftc.teamcode.Teleop.Subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;

import java.io.Serializable;


@Config
public class Intake {
    public DcMotorEx intakeMotor;
    // bottom w/ noodles or pasta
    public CRServo roller1,roller2;
    // roller1 middle, roller 2 top
    public static double power = .7;
    public static double betterPower = 1;

    DigitalChannel breakBeam;


    public Intake (OpMode opMode){
        intakeMotor = opMode.hardwareMap.get(DcMotorEx.class, "intake");
        roller1 = opMode.hardwareMap.get(CRServo.class,"roller1");
        roller2 = opMode.hardwareMap.get(CRServo.class, "roller2");
        intakeMotor.setDirection(DcMotorEx.Direction.REVERSE);

    }

    public void intake_without_sense(){
        intakeMotor.setPower(power);
        roller1.setPower(betterPower);
    }

    public void intakeWithSensor() {
        while (getSensorState()) {
            intakeMotor.setPower(power);
            roller1.setPower(betterPower);
            roller2.setPower(betterPower);
        }
        if (!(getSensorState())) {
//            toilet3.set(0.0);
            intakeMotor.setPower(power);
//            toilet2.set(betterPower);
        }

    }

    public Action actionIntake(){
        return new ParallelAction(
                new InstantAction(() -> intakeMotor.setPower(power))
//                new InstantAction(() -> toilet2.set(betterPower))
                //new InstantAction(() -> toilet3.set(betterPower))
        );
    }
    public Action actionReverseIntake(){
        return new ParallelAction(
                new InstantAction(()->intakeMotor.setPower(-power))
                //new InstantAction(()->toilet3.set(-betterPower)),
//                new InstantAction(()->toilet2.set(-betterPower))
        );
    }

    public void reverseIntake(){
        intakeMotor.setPower(-power);
//        toilet3.set(-betterPower);
//        toilet2.set(-betterPower);
    }

    public void stopIntake(){
        intakeMotor.setPower(0);
        roller2.setPower(0);
        roller1.setPower(0);
    }


    public boolean getSensorState(){
        return breakBeam.getState();
        // true= unbroken
        // false = broken
    }

}
