package org.firstinspires.ftc.teamcode.Teleop.Subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.arcrobotics.ftclib.controller.PIDController;

import org.firstinspires.ftc.teamcode.Teleop.Bot;


@Config

public class Turret {
    public static double p=0,i=0,d=0;
    public static double basePower = 0.1, powerMin = 0.05;
    public double setPoint =0;
    private final PIDController controller;

    public boolean isManual = false;

    public static double manualPower;
    private final MotorEx turretMotor;
    private double hardwareLimit;
    private double ticksPerDegree;
    private final double gearRatio = 141.1/10;
    public Pose2d pose;

    //all values in inches! NEED TO TUNE THIS
    private final double TURRET_BACK_OFFSET = 9;

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
    public int degreesToTicks(double degrees) {
        //formula: (TPR * gear_ratio / 360)
        return ((int)(turretMotor.getCPR() * gearRatio)/360);
    }

    public double wrapAround(double angle) {
        angle %= 360;
        if (angle <= -180) angle += 360;
        if (angle > 180) angle -= 360;
        return angle;
    }

    public void runToAngle(double angle) {
        int ticks = degreesToTicks(wrapAround(angle));
        runTo(ticks);
    }

    //AutoAim (Returns angle we need to feed into RunToAngle())
    public double autoAimField(Pose2d targetPose) {
        pose = Bot.drive.localizer.getPose();

        //Robot Heading
        double botHeading = Math.toDegrees(pose.heading.log());

        //Turret Positions
        double turretX = pose.position.x - TURRET_BACK_OFFSET * Math.cos(Math.toRadians(botHeading));
        double turretY = pose.position.y - TURRET_BACK_OFFSET * Math.sin(Math.toRadians(botHeading));

        //Vector from Turret to Target
        double dx = targetPose.component1().x - turretX;
        double dy = targetPose.component1().y - turretY;

        //Turret Angle
        double theta = Math.toDegrees(Math.atan2(dy, dx));

        return theta;
    }

    public void runManual(double manual) {
        if (manual > powerMin || manual < -powerMin) {
            isManual = true;
            manualPower = manual;
        } else {
            manualPower = 0;
            isManual = false;
        }
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

        //PID CONTROL
        if (!isManual) {
            //PID CONTROL:
            controller.setSetPoint(setPoint);
            turretMotor.set(basePower * controller.calculate(turretMotor.getCurrentPosition()));
        } else {
            //MANUAL MODE:
            turretMotor.set(manualPower);
            controller.setSetPoint(turretMotor.getCurrentPosition());
        }

        //OBELISK


    }
}
}
