package org.firstinspires.ftc.teamcode.Teleop.Subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.arcrobotics.ftclib.controller.PIDController;


@Config

public class Turret {
    public static double p=0,i=0,d=0;
    public static double basePower = 0.1;
    public double setPoint =0;
    private final PIDController controller;

    public boolean isManual = false;

    public static double manualPower;
    private final MotorEx turretMotor;
    private double hardwareLimit;
    private double ticksPerDegree;
    private final double gearRatio = 141.1/10;


    public Turret (OpMode opMode){
        //set controller PID
        controller = new PIDController(p,i,d);

//        controller.setTolerance(tolerance);

        //turretMotor default settings
        turretMotor = new MotorEx(opMode.hardwareMap, "turret", Motor.GoBILDA.RPM_1150);
        turretMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        turretMotor.setRunMode(Motor.RunMode.RawPower);
        turretMotor.setInverted(false);

        //tune HardwareLimit
        hardwareLimit = 0;
    }

    // takes in ticks
    public void runTo(int t){
        setPoint = t;
    }

    //Convert degrees to ticks to use for autoAim
    public double degreesToTicks(double degrees) {
        //formula: (TPR * gear_ratio / 360)
        return (turretMotor.getCPR() * gearRatio)/360;
    }

    public double wrapAround(double angle) {
        angle %= 360;
        if (angle <= -180) angle += 360;
        if (angle > 180) angle -= 360;
        return angle;
    }

    public void runToAngle(double angle) {
        double ticks = degreesToTicks(wrapAround(angle));
        setPoint = ticks;
    }




    public void periodic(){
        controller.setPID(p,i,d);

        //check that PID not going over the hardware limit so it doesn't crash out.

        if (Math.abs(turretMotor.getCurrentPosition()) > hardwareLimit) {
            if (turretMotor.getCurrentPosition() > 0) {
                controller.setSetPoint(hardwareLimit - 100);
            } else {
                controller.setSetPoint(100 - hardwareLimit);
            }

        if (!isManual) {
            //PID CONTROL:
            controller.setSetPoint(setPoint);
            turretMotor.set(basePower * controller.calculate(turretMotor.getCurrentPosition()));
        } else {
            //MANUAL MODE:
            turretMotor.set(manualPower);
            controller.setSetPoint(turretMotor.getCurrentPosition());
        }
    }
}
}
