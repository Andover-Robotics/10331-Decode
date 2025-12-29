package org.firstinspires.ftc.teamcode.Teleop;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Teleop.Subsystems.AprilTag;
import org.firstinspires.ftc.teamcode.Teleop.Subsystems.Hood;
import org.firstinspires.ftc.teamcode.Teleop.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Teleop.Subsystems.Shooter;
import org.firstinspires.ftc.teamcode.Teleop.Subsystems.Turret;
import org.firstinspires.ftc.vision.VisionPortal;

public class BotTest {
    public Intake intake;
    public static BotTest instance;
    public Hood hood;
    public AprilTag aprilTag;
    public Shooter shooter;
    public Turret turret;
    public VisionPortal visionPortal;
    public OpMode opMode;
    public boolean fieldCentricRunMode = false;
    public MotorEx fl, fr, bl, br;



    public BotTest(OpMode opMode) {
        this.opMode = opMode;
        this.turret = new Turret(opMode);
        try {
            fieldCentricRunMode = false;
        } catch (Exception e) {
            fieldCentricRunMode = false;
        }

    }

    public static BotTest getInstance(OpMode opMode) {
        if (instance == null) {
            return instance = new BotTest(opMode);
        }
        instance.opMode = opMode;
        return instance;
    }

//    public Action shootOne(){
//        return new ParallelAction(
//                new InstantAction(()->hood.hoodServo.setPosition(hoodServoPos())),
//                new InstantAction(()->shooter.setTargetRPM(5000)),
//                new InstantAction(()->intake.toilet3.set(1))
//        );
//    }

    public double hoodServoPos() { // assuming function for hood angle: distance is linear for now
        return 1 * (aprilTag.getBearing()) + 50; //PLACEHOLDER returns angle in a servo pos
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


    //drive code
//    public void driveRobotCentric(double strafeSpeed, double forwardBackSpeed, double turnSpeed) {
//        double[] speeds = {
//                (forwardBackSpeed - strafeSpeed + turnSpeed),
//                (forwardBackSpeed + strafeSpeed - turnSpeed),
//                (-forwardBackSpeed - strafeSpeed - turnSpeed),
//                (-forwardBackSpeed + strafeSpeed + turnSpeed)
//        };
//        double maxSpeed = 0;
//        for (int i = 0; i < 4; i++) {
//            maxSpeed = Math.max(maxSpeed, speeds[i]);
//        }
//        if (maxSpeed > 1) {
//            for (int i = 0; i < 4; i++) {
//                speeds[i] /= maxSpeed;
//            }
//        }
//        fl.set(speeds[0]);
//        fr.set(speeds[1]);
//        bl.set(-speeds[2]);
//        br.set(-speeds[3]);
//    }

    //multi-subsystem methods here


}
