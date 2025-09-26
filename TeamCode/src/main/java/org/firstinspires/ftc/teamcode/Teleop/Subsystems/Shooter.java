package org.firstinspires.ftc.teamcode.Teleop.Subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

@Config
public class Shooter {

    // need to write velocity PIDF here
    // need to figure out how to do that
    // take target velocity - actual velocity for error
    // err*kP for proportionality
    // ff = kF * targetV
    // ff + err(pid) = shooterPower
    public final MotorEx shooter;
    public static double p=0.0,i=0.0,d=0.0,f=0.0;
    private final PIDController controller;
    private static double targetRPM = 0.0;
    public double RPM = 0.0;
    public double shooterPower = 0.0;

    public static double toleranceRPM = 40.0;   // speed window


    public Shooter(OpMode opMode){
        shooter = new MotorEx(opMode.hardwareMap,"shooter", Motor.GoBILDA.BARE);
        shooter.setRunMode(Motor.RunMode.RawPower);
        shooter.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);
        controller = new PIDController(p,i,d);
    }
    public void periodic(){
        controller.setPID(p,i,d);
        RPM = shooter.getVelocity() /28 *60;// follows formula for rps (tps/tpr) * 60 for mins
        double ff = f*targetRPM; // feedforward
        double pid = controller.calculate(RPM, targetRPM);//error
        double pRPM = p * pid; // gets product of p constant and error
        shooterPower = pRPM + ff; // makes sure that the rpm has the feedforward floor

        if (Math.abs(targetRPM) < 1e-3) {
            shooterPower = 0.0;
        } else {
            double s = Math.signum(targetRPM);
            shooterPower = s * Math.max(Math.abs(shooterPower), 0);
        }
        shooterPower = checkPower(shooterPower,1.0,0);
        shooter.set(shooterPower);
    }

public double getTargetRPM(){
        return targetRPM;
}
public double getRPM() {
    return RPM;
}
public double getShooterPower() {
    return shooterPower;
}
public static double checkPower(double power,double maxV, double minV) {
        return Math.max(minV,Math.min(maxV,power));
    }


    public boolean atSpeed() { return Math.abs(targetRPM - getRPM()) <= toleranceRPM; }

}


