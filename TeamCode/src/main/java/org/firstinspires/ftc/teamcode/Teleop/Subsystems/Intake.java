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
    // bottom w/ noodles or pasta
    // roller1 middle, roller 2 top
    public static double power = .7;
    public Servo gate;



    DigitalChannel breakBeam;


    public Intake (OpMode opMode){
        intakeMotor = opMode.hardwareMap.get(DcMotorEx.class, "intake");
        intakeMotor.setDirection(DcMotorEx.Direction.REVERSE);

    }
    public Action openGate(double dt){
        return new SequentialAction(
               new InstantAction(()->gate.setPosition(1)),
                new SleepAction(dt),
                new InstantAction(()->gate.setPosition(0))

        );
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
    }


    public boolean getSensorState(){
        return breakBeam.getState();
        // true= unbroken
        // false = broken
    }

}
