package org.firstinspires.ftc.teamcode.Teleop.Subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class Intake {
    public DcMotorEx intakeMotor;
    public static double power = .7;

    public Servo gate1;
    public Servo gate2;
    public DcMotorEx secondIntake;
    DigitalChannel breakBeam;


    public Intake (OpMode opMode){
        intakeMotor = opMode.hardwareMap.get(DcMotorEx.class, "intake");
        secondIntake = opMode.hardwareMap.get(DcMotorEx.class, "intake2");
        gate1 = opMode.hardwareMap.servo.get("gate1");
        gate2 = opMode.hardwareMap.servo.get("gate2");

        intakeMotor.setDirection(DcMotorEx.Direction.REVERSE);
        gate2.setDirection(Servo.Direction.REVERSE);


    }


    public void closeGate(){
        gate1.setPosition(0.2);
        gate2.setPosition(0.3);
    }
    public void openGate(){
        gate1.setPosition(0);
        gate2.setPosition(0);
    }

    public void intake_without_sense(double power){
        closeGate();
        intakeMotor.setPower(power);
    }
    public void intake_without_sense(){
        closeGate();
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
                new InstantAction(() -> intakeMotor.setPower(power))
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
        secondIntake.setPower(0);
    }


    public boolean getSensorState(){
        return breakBeam.getState();
        // true= unbroken
        // false = broken
    }

}
