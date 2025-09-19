package org.firstinspires.ftc.teamcode.Teleop;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Teleop.Subsystems.AprilTag;
import org.firstinspires.ftc.teamcode.Teleop.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Teleop.Subsystems.Shooter;
import org.firstinspires.ftc.vision.VisionPortal;

public class Bot {
    public Intake intake;
    public static Bot instance;
    public Shooter shooter;
    public AprilTag aprilTag;
    public VisionPortal visionPortal;
    public OpMode opMode;
    public boolean fieldCentricRunMode = false;
    public MotorEx fl,fr,bl,br;
    public Bot(OpMode opMode){
        this.aprilTag = new AprilTag(opMode);
        this.opMode = opMode;
        this.shooter = new Shooter(opMode);
        this.intake = new Intake(opMode);
        try {
            fieldCentricRunMode = false;
        } catch (Exception e) {
            fieldCentricRunMode = false;
        }
        fl = new MotorEx(opMode.hardwareMap, "fl", Motor.GoBILDA.RPM_435);
        fr = new MotorEx(opMode.hardwareMap, "fr", Motor.GoBILDA.RPM_435);
        bl = new MotorEx(opMode.hardwareMap, "bl", Motor.GoBILDA.RPM_435);
        br = new MotorEx(opMode.hardwareMap, "br", Motor.GoBILDA.RPM_435);
    }
    public static Bot getInstance(OpMode opMode) {
        if (instance == null) {
            return instance = new Bot(opMode);
        }
        instance.opMode = opMode;
        return instance;
    }

    //drive code
    public void driveRobotCentric(double strafeSpeed, double forwardBackSpeed, double turnSpeed) {
        double[] speeds = {
                (forwardBackSpeed - strafeSpeed + turnSpeed),
                (forwardBackSpeed + strafeSpeed - turnSpeed),
                (-forwardBackSpeed - strafeSpeed - turnSpeed),
                (-forwardBackSpeed + strafeSpeed + turnSpeed)
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
