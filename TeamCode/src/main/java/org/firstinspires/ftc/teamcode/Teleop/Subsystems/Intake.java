package org.firstinspires.ftc.teamcode.Teleop.Subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.arcrobotics.ftclib.hardware.motors.CRServo;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import com.qualcomm.robotcore.hardware.DigitalChannel;

import org.firstinspires.inspection.InspectionState;


@Config
public class Intake {
    public DcMotorEx intakeMotor;
    // bottom w/ noodles or pasta
    public CRServo toilet2;
    // middle toilet roll--does not stop
    public CRServo toilet3;
    // top toilet roll--stops
    public static double power = 0.0;
    public static double betterPower = 0.0;

    DigitalChannel breakBeam;


    public Intake (OpMode opMode){
        intakeMotor.setDirection(DcMotorEx.Direction.FORWARD);
        intakeMotor = opMode.hardwareMap.get(DcMotorEx.class, "actionIntake motor");
        toilet3 = opMode.hardwareMap.get(CRServo.class,"toilet roll 3");
        toilet2 = opMode.hardwareMap.get(CRServo.class, "toilet 2");
        breakBeam = opMode.hardwareMap.get(DigitalChannel.class, "BreakBeam");
    }

    public void intake_without_sense(){
        intakeMotor.setDirection(DcMotorEx.Direction.FORWARD);
        intakeMotor.setPower(power);
        toilet3.set(betterPower);
        toilet2.set(betterPower);
    }

    public void intakeWithSensor() {
        while (getSensorState()) {
            intakeMotor.setPower(power);
            toilet3.set(betterPower);
            toilet2.set(betterPower);
        }
        if (!(getSensorState())) {
            toilet3.set(0.0);
            intakeMotor.setPower(power);
            toilet2.set(betterPower);
        }

    }

    public Action actionIntake(){
        return new ParallelAction(
                new InstantAction(() -> intakeMotor.setPower(power)),
                new InstantAction(() -> toilet2.set(betterPower)),
                new InstantAction(() -> toilet3.set(betterPower))
        );
    }
    public Action actionStopIntake(){
        return new ParallelAction(
                new InstantAction(()->intakeMotor.setPower(0)),
                new InstantAction(()->toilet3.set(-betterPower)),
                new InstantAction(()->toilet2.set(-betterPower))
        );
    }

    public void reverseIntake(){
        intakeMotor.setPower(-power);
        toilet3.set(-betterPower);
        toilet2.set(-betterPower);
    }

    public void stopIntake(){
        intakeMotor.setPower(0);
        toilet3.set(0);
        toilet2.set(0);
    }


    public boolean getSensorState(){
        return breakBeam.getState();
        // true= unbroken
        // false = broken
    }

}
