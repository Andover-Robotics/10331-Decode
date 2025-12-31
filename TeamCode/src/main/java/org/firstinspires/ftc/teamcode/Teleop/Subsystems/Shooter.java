package org.firstinspires.ftc.teamcode.Teleop.Subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

@Config
public class Shooter {

    // need to write velocity PIDF here
    // need to figure out how to do that
    // take target velocity - actual velocity for error
    // err*kP for proportionality
    // ff = kF * targetV
    // ff + err(pid) = shooterPower

    public final MotorEx shooter;
    //y=-0.000860551x^{3}+0.278353x^{2}-11.54167x+3650.11204 regression values
    public double shooterA = -0.000860551, shooterB = 0.278353, shooterC = -11.54167, shooterD = 3650.11204;
    public final MotorEx shooter2;
    public static double p = 0.00025, i = 0.0, d = 0.0, f = 0.000195;
    private final PIDController controller;
    public static int targetRPM = 0;
    public static int target;

    public double RPM = 0.0;
    public double shooterPower = 0.0;


    public static double toleranceRPM = 60.0;   // speed window


    public Shooter(OpMode opMode) {
        shooter = new MotorEx(opMode.hardwareMap, "shooter", Motor.GoBILDA.BARE);
        shooter.setRunMode(Motor.RunMode.RawPower);
        shooter.setInverted(true);
        shooter.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);
        shooter2 = new MotorEx(opMode.hardwareMap, "shooter2", Motor.GoBILDA.BARE);
        shooter2.setRunMode(Motor.RunMode.RawPower);
        shooter2.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);
        controller = new PIDController(p, i, d);


    }

    public void setPower(double power) {
        shooter.set(power);
        shooter2.set(power);
    }

    public void runShooter(double s) {
        shooter.set(s);
    }

    public void periodic() {
        controller.setPID(p, i, d);

        RPM = shooter.getVelocity() / 28 * 60;// follows formula for rps (tps/tpr) * 60 for mins
        double ff = f * targetRPM; // feedforward
        double pid = controller.calculate(RPM, targetRPM);//error
        // gets product of p constant and error

        shooterPower = pid + ff; // makes sure that the rpm has the feedforward floor

//        if (Math.abs(targetRPM) < 1e-3) {
//            shooterPower = 0.0;
//        } else {
//            double s = Math.signum(targetRPM);
//            shooterPower = s * Math.max(Math.abs(shooterPower), 0);
//        }
        shooterPower = checkPower(shooterPower, 1.0, 0);
        setPower(shooterPower);
    }

    public void reset() {
        targetRPM = 0;
        shooterPower = 0;
        controller.reset();
    }

    public void setTargetRPM(int t) {// probably temp but I cant remember how to do ts
        targetRPM = t;

    }


    public double getTargetRPM() {
        return targetRPM;
    }

    public double getRPM() {
        return RPM;
    }

    public double getShooterPower() {
        return shooterPower;
    }

    public static double checkPower(double power, double maxV, double minV) {
        return Math.max(minV, Math.min(maxV, power));
    }


    public boolean atSpeed() {
        return Math.abs(targetRPM - getRPM()) <= toleranceRPM;
    }


    public double getCurrent() {
        return shooter.motorEx.getCurrent(CurrentUnit.MILLIAMPS);

    }
}

