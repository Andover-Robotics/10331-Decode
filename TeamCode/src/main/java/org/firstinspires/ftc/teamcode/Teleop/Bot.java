package org.firstinspires.ftc.teamcode.Teleop;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Teleop.Subsystems.AprilTag;
import org.firstinspires.ftc.teamcode.Teleop.Subsystems.Hood;
import org.firstinspires.ftc.teamcode.Teleop.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Teleop.Subsystems.Shooter;
import org.firstinspires.ftc.vision.VisionPortal;

public class Bot {
    public Intake intake;
    public static Bot instance;
    public Hood hood;

    public AprilTag aprilTag;
    public Shooter shooter;
    public VisionPortal visionPortal;
    public OpMode opMode;
    public boolean fieldCentricRunMode = false;
    public MotorEx fl, fr, bl, br;
    public static double dt;

    public enum BotState {
        AUTO,
        MANUAL
    }


    public Bot(OpMode opMode) {
        this.aprilTag = new AprilTag(opMode);
        this.opMode = opMode;
        this.shooter = new Shooter(opMode);
        this.hood = new Hood(opMode);
        this.intake = new Intake(opMode);
        try {
            fieldCentricRunMode = false;
        } catch (Exception e) {
            fieldCentricRunMode = false;
        }
        fl = new MotorEx(opMode.hardwareMap, "perp", Motor.GoBILDA.RPM_435);
        fr = new MotorEx(opMode.hardwareMap, "fr", Motor.GoBILDA.RPM_435);
        bl = new MotorEx(opMode.hardwareMap, "par", Motor.GoBILDA.RPM_435);
        br = new MotorEx(opMode.hardwareMap, "br", Motor.GoBILDA.RPM_435);
        fl.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        fr.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        bl.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        br.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);


    }

    public static Bot getInstance(OpMode opMode) {
        if (instance == null) {
            return instance = new Bot(opMode);
        }
        instance.opMode = opMode;
        return instance;
    }

    public void switchAlliance(){
        if (aprilTag.targetAllianceId==20){
            aprilTag.targetAllianceId = 24;
        }
        else {
            aprilTag.targetAllianceId = 20;
        }
    }



    public Action shootSetup(){
         return new ParallelAction(
                 new InstantAction(()-> intake.openGate()),
                 new InstantAction(()->intake.secondIntake.setPower(1))
         );
    }
    public Action actionShoot(){
        return new SequentialAction(
                new InstantAction(()->shooter.setTargetRPM((int)calculateRPM()+75)),//may need to not round here in the future
                new SleepAction(0.3),
                shootSetup()
                );
    }

    public Action actionTestShoot(){
        return new SequentialAction(
                new InstantAction(()->shooter.setTargetRPM(4000)),//may need to not round here in the future
                new SleepAction(0.3),
                shootSetup()
        );
    }


    public double calculateRPM(){
        double dist = aprilTag.calcAccurateDis();
        return (shooter.shooterA * Math.pow(dist,3)) + (shooter.shooterB*Math.pow(dist,2))+ (shooter.shooterC*dist)+shooter.shooterD;
    }

    public class actionPeriodic implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            shooter.periodic();
            return true;
        }
    }
    public Action actionPeriodic() { // for auto to work
        return new actionPeriodic();
    }


//        drive code
    public void driveRobotCentric(double strafeSpeed, double forwardBackSpeed, double turnSpeed) {
        double[] speeds = {
                (forwardBackSpeed - strafeSpeed - turnSpeed),
                (-forwardBackSpeed - strafeSpeed - turnSpeed),
                (-forwardBackSpeed - strafeSpeed + turnSpeed),
                (forwardBackSpeed - strafeSpeed + turnSpeed)
        };
        double maxSpeed = 0;
        for (int i = 0; i < 4; i++) {
            maxSpeed = Math.max(maxSpeed, speeds[i]);
        }
        if (maxSpeed > 1) {
            for (int i = 0; i < 4; i++) {
                speeds[i] /= maxSpeed;
            }
        }
        fl.set(speeds[0]);
        fr.set(speeds[1]);
        bl.set(-speeds[2]);
        br.set(-speeds[3]);
    }

        //multi-subsystem methods here


}
