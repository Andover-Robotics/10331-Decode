package org.firstinspires.ftc.teamcode.Teleop;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Auto.miscRR.MecanumDrive;
import org.firstinspires.ftc.teamcode.Teleop.Subsystems.AprilTag;
import org.firstinspires.ftc.teamcode.Teleop.Subsystems.Hood;
import org.firstinspires.ftc.teamcode.Teleop.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Teleop.Subsystems.Shooter;
import org.firstinspires.ftc.teamcode.Teleop.Subsystems.Turret;
import org.firstinspires.ftc.vision.VisionPortal;

import java.util.ArrayList;
import java.util.List;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
@Config
public class Bot {
    public Intake intake;
    public static Bot instance;
    public Hood hood;

    public AprilTag aprilTag;
    public Shooter shooter;
    public Turret turret;
    public VisionPortal visionPortal;
    public OpMode opMode;
    public boolean fieldCentricRunMode = false;
    public MotorEx fl, fr, bl, br;
    public static double shootSleep=0.2, shootDelay=0.7;
    public boolean isRed;
    public boolean recoilIsTrue;


    //Stored poses
    //---------------------------------------------------------
    public static Pose2d storedPose = new Pose2d(0,0,0);
    public static Vector2d goalPose = new Vector2d(65,-60);// init with red
    public static Pose2d resetPose = new Pose2d(-63,-63,Math.toRadians(-90)); // change when we figure out where we want to reset



    public static MecanumDrive drive;

    public enum BotState {
        AUTO,
        MANUAL
    }

    public IMU imu;

    public Bot(OpMode opMode) {
        this.aprilTag = new AprilTag(opMode);
        this.opMode = opMode;
        this.shooter = new Shooter(opMode);
        this.hood = new Hood(opMode);
        this.intake = new Intake(opMode);
        this.turret = new Turret(opMode);
        try {
            fieldCentricRunMode = false;
        } catch (Exception e) {
            fieldCentricRunMode = false;
        }
        drive = new MecanumDrive(opMode.hardwareMap, storedPose);
        fl = new MotorEx(opMode.hardwareMap, "fl", Motor.GoBILDA.RPM_435);
        fr = new MotorEx(opMode.hardwareMap, "perp", Motor.GoBILDA.RPM_435);
        bl = new MotorEx(opMode.hardwareMap, "par", Motor.GoBILDA.RPM_435);
        br = new MotorEx(opMode.hardwareMap, "br", Motor.GoBILDA.RPM_435);


        imu = opMode.hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                        RevHubOrientationOnRobot.UsbFacingDirection.UP
                )
        );

        imu.initialize(parameters);

//        IMU.Parameters parameters = new IMU.Parameters(
//                new RevHubOrientationOnRobot()
//        );
//        parameters.mode                = BNO055IMU.SensorMode.IMU;
//        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
//        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
//        parameters.loggingEnabled      = false;
//        imu = opMode.hardwareMap.get(IMU.class, "imu");
//        imu.initialize(parameters);

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


    public void updatePoses(){
        if (isRed){
            goalPose = new Vector2d(goalPose.x,Math.abs(goalPose.y));
            resetPose = new Pose2d(resetPose.component1().x, Math.abs(resetPose.component1().y), Math.abs(resetPose.heading.log()));
        }
        else {
            if(goalPose.y!=60) goalPose = new Vector2d(goalPose.x,(-1*goalPose.y));
            resetPose = new Pose2d(resetPose.component1().x, -1*(resetPose.component1().y), -1*(resetPose.heading.log()));
        }
    }


    //Add useStoredPose
    public static void useStoredPose() {
        drive.localizer.setPose(storedPose);
    }


    public void switchAlliance(){
        if (aprilTag.targetAllianceId==20){
            aprilTag.targetAllianceId = 24;
        }
        else {
            aprilTag.targetAllianceId = 20;
        }
    }


    //count current balls shot indexing from 0, using the current spike when shot
    //increment a counter and lower height of the hood
    //public void recoil(){
        int ballsShot =0;
        //if (shooter.getCurrent() >9000){ //arbitrary current number need to test
            //ballsShot++;
        //}

        /*switch (ballsShot){
            case 0:
                break;
            case 1:
                if(shooter.getCurrent() >9000){hood.goToHood(0.2); ballsShot++;}
                break;
            case 2:
                if(shooter.getCurrent() >9000){hood.goToHood(0.1);}
                break;
            default:
                hood.goToHood(0.3);
        }*/
    //}

    public Action shootSetup(){
         return new ParallelAction(
                 new InstantAction(()-> intake.openGate())
         );
    }
    public Action actionShoot(){
        return new SequentialAction(
                new InstantAction(()->shooter.enableShooter(true)),//may need to not round here in the future
                new SleepAction(0.3),
                new InstantAction(()-> intake.openGate())

                );
    }

    public Action actionStopShoot(){
        return new SequentialAction(
                new InstantAction(()->shooter.enableShooter(false)),
                new InstantAction(()-> intake.closeGate())
        );
    }

    public Action actionShootGate(){
        return new SequentialAction(
                new InstantAction(()->shooter.enableShooter(true)),
                new SleepAction(0.35),
                new InstantAction(() -> intake.openGate()),
                new SleepAction(shootSleep),
                new InstantAction(() -> intake.closeGate()),
                new SleepAction(shootDelay),
                new InstantAction(() -> intake.openGate()),
                new SleepAction(shootSleep),
                new InstantAction(() -> intake.closeGate()),
                new SleepAction(shootDelay),
                new InstantAction(() -> intake.openGate()),
                new SleepAction(shootSleep),
                new InstantAction(() -> intake.closeGate())
        );
    }
    public Action actionShootGateTest(){
        return new SequentialAction(
                new InstantAction(()->shooter.setTargetRPM(4500)),
                new SleepAction(0.35),
                new InstantAction(() -> intake.openGate()),
                new SleepAction(shootSleep),
                new InstantAction(() -> intake.closeGate()),
                new SleepAction(shootDelay),
                new InstantAction(() -> intake.openGate()),
                new SleepAction(shootSleep),
                new InstantAction(() -> intake.closeGate()),
                new SleepAction(shootDelay),
                new InstantAction(() -> intake.openGate()),
                new SleepAction(shootSleep),
                new InstantAction(() -> intake.closeGate())
        );
    }

    public Action actionTestShoot(){
        return new SequentialAction(
                new InstantAction(()->shooter.setTargetRPM(4000)),//may need to not round here in the future
                new SleepAction(0.3),
                shootSetup()

        );
    }


    public static int regressionRPM(double dist) {
        // shooterA= -0.000860551,shooterB= 0.278353,shooterC= -11.54167,shooterD=3650.11204;
        double shooterAComponent = Shooter.shooterA * Math.pow(dist,3);
        double shooterBComponent = Shooter.shooterB * Math.pow(dist,2);
        double shooterCComponent = Shooter.shooterC * dist;
        double shooterDComponent = Shooter.shooterD;

        double regression = shooterAComponent + shooterBComponent + shooterCComponent + shooterDComponent ;

        return (int)regression + 75;
    }

    public class actionPeriodic implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            aprilTag.findAprilTag();
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
                (-forwardBackSpeed +strafeSpeed +    turnSpeed),
                (forwardBackSpeed + strafeSpeed + turnSpeed),
                (forwardBackSpeed + strafeSpeed - turnSpeed),
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

    public List<MotorEx> returnMotors(){
        List<MotorEx> motorList = new ArrayList<>();
        motorList.add(fl);
        motorList.add(bl);
        motorList.add(br);
        motorList.add(fr);
        return motorList;

    }

    public void prepTeleop(){
        bl.setInverted(true);
        fr.setInverted(true);
        intake.closeGate();
        hood.hoodServo.setPosition(0.3);
        Hood.outtakePos=0.3;
        aprilTag.targetAllianceId=24;

    }public void prepAuto(int alliance, boolean isRed){
        intake.closeGate();
        hood.hoodServo.setPosition(0.3);
        Hood.outtakePos=0.3;
        aprilTag.targetAllianceId=alliance;
        this.isRed = isRed;
        updatePoses();

    }

    public void setStoredPose(Pose2d sp){
        storedPose=sp;
    }

    //current bot pos to stored pos at  end of auto

//
//    public BNO055IMU returnIMU(){
//        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
//        parameters.mode                = BNO055IMU.SensorMode.IMU;
//        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
//        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
//        parameters.loggingEnabled      = false;
//        BNO055IMU imu = opMode.hardwareMap.get(BNO055IMU.class, "imu");
//        imu.initialize(parameters);
//        return imu;
 //   }




}
