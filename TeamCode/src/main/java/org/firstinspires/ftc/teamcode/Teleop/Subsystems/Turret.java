package org.firstinspires.ftc.teamcode.Teleop.Subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.arcrobotics.ftclib.controller.PIDController;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.Teleop.Bot;

import java.io.DataInput;


@Config

public class Turret {

    /*TODO: implement proper wraparound for turret
        implement testing code

     */
    public static double p=0,i=0,d=0,tX,tY,tA;
    public static double basePower = 0.1, powerMin = 0.05;
    public double setPoint =0;
    private final PIDController controller;

    public boolean isManual = false;

    public static double manualPower;
    private final MotorEx turretMotor;
    public final double a1 = 20; //angle of physical limelight
    public final double HEIGHT_OFFSET = 16.625;
    private double hardwareLimit;
    private double ticksPerDegree;
    private final double gearRatio = 141.1/10;
    public Pose2d pose;

    public static Pose3D botPose = new Pose3D(new Position(DistanceUnit.INCH,0,0,0,0),new YawPitchRollAngles(AngleUnit.DEGREES,0,0,0,0));

    public double distance,lldistance;

    public LLResult llResult;



    public Limelight3A ll;

    //all values in inches! NEED TO TUNE THIS
    private final double TURRET_BACK_OFFSET = 9;

    public Turret (OpMode opMode){

        //set controller PID
        controller = new PIDController(p,i,d);

//        controller.setTolerance(tolerance);

        ll = opMode.hardwareMap.get(Limelight3A.class,"Limelight");
        ll.setPollRateHz(100);
        ll.start();

        //turretMotor default settings
        turretMotor = new MotorEx(opMode.hardwareMap, "turret", Motor.GoBILDA.RPM_1150);
        turretMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        turretMotor.setRunMode(Motor.RunMode.RawPower);
        turretMotor.setInverted(false);

        //tune HardwareLimit
        hardwareLimit = 0;
    }

    // takes in ticks
    public void runTo(int t){ // takes in ticks
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

        distance= Math.sqrt(dx*dx+dy*dy);

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

    public void periodic() {
        controller.setPID(p, i, d);

        //check that PID not going over the hardware limit so it doesn't crash out.
        if (Math.abs(turretMotor.getCurrentPosition()) > hardwareLimit) {
            if (turretMotor.getCurrentPosition() > 0) {
                setPoint = (hardwareLimit - 100);
            } else {
                setPoint = (100 - hardwareLimit);
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

        public void periodic2() { // will be failsafe for turret moving based on ll result
            controller.setPID(p,i,d);
            llResult = ll.getLatestResult();
            if (llResult.isValid() && llResult!=null){
                botPose = llResult.getBotpose_MT2();
                double dx = Bot.goalPose.x - botPose.getPosition().x;
                double dy = Bot.goalPose.y - botPose.getPosition().y;
                tX=llResult.getTx();
                lldistance = Math.sqrt(dx*dx+dy*dy);
            }

            runToAngle(tX);
            controller.calculate(turretMotor.getCurrentPosition());



            double targetAngle =0;
            double power = controller.calculate(degreesToTicks(tX),targetAngle);
            turretMotor.set(power);


        }

        public void relocalizeRobot(){
            Bot.drive.localizer.setPose(new Pose2d(botPose.getPosition().toUnit(DistanceUnit.INCH).x,botPose.getPosition().toUnit(DistanceUnit.INCH).y, Math.toRadians(botPose.getOrientation().getYaw())));

        }
    }

