package org.firstinspires.ftc.teamcode.Teleop.Subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class Intake {
    public DcMotorEx intakeMotor;
    public static double power = .7;

    public Servo gate1;

    DigitalChannel breakBeam;


    public Intake (OpMode opMode){
        intakeMotor = opMode.hardwareMap.get(DcMotorEx.class, "intake");
        gate1 = opMode.hardwareMap.servo.get("gate1");

        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        intakeMotor.setDirection(DcMotorEx.Direction.REVERSE);
    }


    public void closeGate(){
        gate1.setPosition(0.15);
    }
    public void openGate(){
        gate1.setPosition(0.31);
    }

    public void intake_without_sense(double power){
        intakeMotor.setPower(power);
    }
    public void intake_without_sense(){
        intakeMotor.setPower(power);
    }

    public void intakeWithSensor() {
        while (getSensorState()) {
            intakeMotor.setPower(power);
        }
        if (!(getSensorState())) {
            intakeMotor.setPower(power);
        }
    }

    public Action actionIntake(){
        return new ParallelAction(
                new InstantAction(() -> intakeMotor.setPower(1))
        );
    }
    public Action actionReverseIntake(){
        return new ParallelAction(
                new InstantAction(()->intakeMotor.setPower(-power))
        );
    }

    public void reverseIntake(){
        intakeMotor.setPower(-power);
    }

    public void stopIntake(){
        intakeMotor.setPower(0);
    }


    public boolean getSensorState(){
        return breakBeam.getState();
        // true= unbroken
        // false = broken
    }

}
