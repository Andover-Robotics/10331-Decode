package org.firstinspires.ftc.teamcode.Teleop.Subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
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



@Config

public class Turret {

    /*TODO: implement proper wraparound for turret
        implement testing code

     */
    public static double p=0.0035,i=0,d=0.00015,tX,tY,tA;
    public static double basePower = 0.1, powerMin = 0.05;
    public static double setPoint = 0;
    private final PIDController controller;

    public boolean isManual = false;
    private final double CPR = 145.1;
    private final double GEAR_RATIO = 102/14;

    public static double manualPower;
    public final MotorEx turretMotor;
    public final double a1 = 20; //angle of physical limelight
    public final double HEIGHT_OFFSET = 16.625;
    private double hardwareLimit;
    private double degPerTick = 360 /(CPR* GEAR_RATIO);
    public Pose2d pose;
    public static Pose3D botPose = new Pose3D(new Position(DistanceUnit.INCH,0,0,0,0),new YawPitchRollAngles(AngleUnit.DEGREES,0,0,0,0));
    public double distance,lldistance,power;

    public LLResult llResult;
    public Limelight3A ll;

    //all values in inches! NEED TO TUNE THIS
    private final double TURRET_BACK_OFFSET = 9;

    public Turret (OpMode opMode){

        //set controller PID
        controller = new PIDController(p,i,d);

//        controller.setTolerance(tolerance);
//
//        ll = opMode.hardwareMap.get(Limelight3A.class,"Limelight");
//        ll.setPollRateHz(100);
//       ll.pipelineSwitch(1); // pipeline index from web UI
//
//        ll.start();



        //turretMotor default settings
        turretMotor = new MotorEx(opMode.hardwareMap, "turret", Motor.GoBILDA.RPM_1150);
        turretMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        turretMotor.setRunMode(Motor.RunMode.RawPower);
        turretMotor.setInverted(false);

        //tune HardwareLimit
//        hardwareLimit = 0;
    }
    public void switchPipeline(int pipe) {
        ll.pipelineSwitch(pipe);
    }

    // takes in ticks
    public void runTo(int t){ // takes in ticks
        setPoint = t;
    }//tested works


    public double wrapAround(double angle) {
       // angle %= 360; // i feel like there might be issue here
        if (angle >= 360) angle -= 360;
        if (angle < 0) angle += 360;
        return angle;
    } //tested works i think may need to change when angles are normalized

    public void runToAngle(double angle) {
        runTo((degreesToTicks(wrapAround(angle))));
    } // tested works i think


    //AutoAim (Returns angle we need to feed into RunToAngle())
    public double autoAimField(Vector2d targetPose) {
        pose = Bot.drive.localizer.getPose();

        //Robot Heading
        double botHeading = Math.toDegrees(pose.heading.log());

        //Turret Positions
        //credit to Lighting:
        double turretX = pose.position.x - TURRET_BACK_OFFSET * Math.cos(Math.toRadians(botHeading));
        double turretY = pose.position.y - TURRET_BACK_OFFSET * Math.sin(Math.toRadians(botHeading));

        //Vector from Turret to Target
        double dx = targetPose.x - turretX;
        double dy = targetPose.y - turretY;

        //Target Angle
        double ccwFieldTarget = Math.toDegrees(Math.atan2(dy, dx));

        distance= Math.sqrt(dx*dx+dy*dy);

        //robot heading is ccw+
        //angle of turret relative to field is ccw+
        //encoder of turret is cw+
        //setting the target angle to the output theta will make it tweak out and go cw to the angle instead of ccw
        //need to convert the output to being cw+ but shortest path

        //partially chatted code may need to rewrite
        //assume -180 relative to robot is 0 so turret should start facing backwards i think it tweaks out if it faces forwards at 0
        double ccwTargetRelToRobot = normDelta(ccwFieldTarget - botHeading);
        double cwTarget = normDelta(-ccwTargetRelToRobot);

        return cwTarget;
    }




//    public void runManual(double manual) {
//        if (manual > powerMin || manual < -powerMin) {
//            isManual = true;
//            manualPower = manual;
//        } else {
//            manualPower = 0;
//            isManual = false;
//        }
//    }

    public void periodic() {
        power =0;
        controller.setPID(p, i, d);

        //check that PID not going over the hardware limit so it doesn't crash out.
//        if (Math.abs(turretMotor.getCurrentPosition()) > hardwareLimit) {
//            if (turretMotor.getCurrentPosition() > 0) {
//                setPoint = (hardwareLimit - 100);
//            } else {
//                setPoint = (100 - hardwareLimit);
//            }
//        }

            // add manual back later but i cant look at ts rn
            //controller.setSetPoint(setPoint);
            power = controller.calculate(turretMotor.getCurrentPosition(),setPoint);
            power = clamp(power,1.0,-1.0);
            turretMotor.set(power);
    } //tested works i think

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
            Pose2d botPose2d = convertPose(botPose);
            Bot.drive.localizer.setPose(botPose2d);
        }

        //getters
        public double getTargetTicks(){return setPoint;}
        public double getCurrentTicks(){return turretMotor.getCurrentPosition();}

        public double getTargetDegrees(){return setPoint*degPerTick;}
        public double getCurrentDegrees(){return turretMotor.getCurrentPosition() * degPerTick;}

        public double getCurrentPower(){return power;}



        // util

    //Convert degrees to ticks to use for autoAim
        public int degreesToTicks(double degrees) {
        //formula: (TPR * gear_ratio / 360)
        return ((int)(degrees*((CPR * GEAR_RATIO)/360)));
        }

        //normalize to ts [−180°,180°) so that shortest paths actually work
        // chatted so might not work; supposedly faster than trig
        public double normDelta(double angle){
        return ((angle + 180) % 360 + 360) % 360 - 180;
    }
        //stackoverflow
//        public double normDelta(double angleDelta) {
//            return Math.toDegrees(Math.atan2(Math.sin(Math.toRadians(angleDelta)), Math.cos(Math.toRadians(angleDelta))));
//        }

        public Pose2d convertPose(Pose3D pose){
        double x = pose.getPosition().toUnit(DistanceUnit.INCH).x;
        double y = pose.getPosition().toUnit(DistanceUnit.INCH).y;
        double heading = Math.toRadians(pose.getOrientation().getYaw());
        return new Pose2d(x,y,heading);
        }

    private double clamp(double power,double maxV, double minV) {
    return Math.max(minV,Math.min(maxV,power));
}
}

