package org.firstinspires.ftc.teamcode.Teleop.Subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
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
    public static double p=0.0035,i=0,d=0.000162,tX,pShort=0.0055,iShort,dShort=0.0002;
    public static double basePower = 0.1, powerMin = 0.05;
    public static double setPoint = 0;
    private final PIDController controller;
    Hood hood; //slime me out

    public boolean isManual = false;
    private final double CPR = 145.1;
    private final double GEAR_RATIO = 102.0/14;

    public static double manualPower;
    public final MotorEx turretMotor;
    public final double a1 = 20; //angle of physical limelight
    public final double HEIGHT_OFFSET = 16.625;
    private double hardwareLimit;
    private double degPerTick = 360 /(CPR* GEAR_RATIO);
    public Pose2d pose;
    public static Pose3D botPose = new Pose3D(new Position(DistanceUnit.INCH,0,0,0,0),new YawPitchRollAngles(AngleUnit.DEGREES,0,0,0,0));
    public PoseVelocity2d botVel;
    public double lldistance,power;
    public static double distance;
    public static double compDistance;

    public static boolean isLocked;

    public LLResult llResult;
    public Limelight3A ll;

    //all values in inches! NEED TO TUNE THIS
    private final double TURRET_BACK_OFFSET = 2.3;
    public double vectorXComp;
    public double vectorYComp;
    public boolean enableVelComp;


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
        //angle =AngleUnit.normalizeDegrees(angle);
       // angle %= 360; // i feel like there might be issue here
        if (angle < 0) angle += 360; //low limit
        if (angle >= 315) angle -= 360; // high limit
//        angle %=360;
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
        double ccwFieldTarget;
        distance = Math.sqrt(dx * dx + dy * dy);
        changeTrackingPose(dx,dy);
        compDistance = Math.sqrt(vectorXComp*vectorXComp+vectorYComp*vectorYComp);
        //TODO: uncomment once the bomboclatt placeholders are gone
        //Target Angle
        if(!enableVelComp) { //might need to move to tracking pose method
             ccwFieldTarget = Math.toDegrees(Math.atan2(dy, dx));
        }
        else{
            ccwFieldTarget = Math.toDegrees(Math.atan2(vectorYComp,vectorXComp));
        }

        //robot heading is ccw+
        //angle of turret relative to field is ccw+
        //encoder of turret is cw+
        //setting the target angle to the output theta will make it tweak out and go cw to the angle instead of ccw
        //need to convert the output to being cw+ but shortest path

        //partially chatted code may need to rewrite
        //assume -180 relative to robot is 0 so turret should start facing backwards i think it tweaks out if it faces forwards at 0
        double ccwTargetRelToRobot = normDelta(ccwFieldTarget - botHeading);
        double cwTarget = normDelta(-ccwTargetRelToRobot);

        return cwTarget +270;
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
        power = 0;
        Bot.drive.updatePoseEstimate();
        Bot.storedPose = Bot.drive.localizer.getPose();
        double error = setPoint - turretMotor.getCurrentPosition();

        if(error*degPerTick>10)controller.setPID(p, i, d);
        else controller.setPID(pShort,iShort,dShort);
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
        if(isLocked) runToAngle(autoAimField(Bot.goalPose));
        //locks forward for drifting if i can't fix
        else { autoAimField(Bot.goalPose);}

        power = controller.calculate(turretMotor.getCurrentPosition(),setPoint);
        power = clamp(power,1.0,-1.0);
        turretMotor.set(power);

    } //tested works i think


        public double airtimeCalc(double dx,double dy){

        final double g = 386.22; // g in in/s^2
        //final double defaultAngle = 33.9;
            //0.25 angle
        final double staticAngle = 41.6;
        //final double hoodGR = 28.0/30.0;
        //final double hoodAngleAdaptive = ((33.9*(hoodAngle/0.7)); 0.7 is cap and shows increase  //assuming axon is 35.5 degrees/0.1 increment gets hood servo angle in degrees and relates to hood's angle
        final double goalHeightBotRel = 20.5; //placeholder -- need to measure (h)
        double horizontalDist = Math.sqrt(dx*dx+dy*dy); // gets horizontal
            //follows trajectory formula for time to find the time it takes for a ball to go from the robot to the goal
            //t^2 = (2/g)*(d*tan(theta)-h)
        double timeSquared = ((2/g)*(horizontalDist*Math.tan(staticAngle))-goalHeightBotRel);
        double time = Math.sqrt(timeSquared);
        if (timeSquared<=0) return 0;//avoids complex solutions
        else return time;

    }
    public void changeTrackingPose(double dx,double dy){
         botVel = Bot.drive.localizer.update();
         double time = airtimeCalc(dx,dy);
         double heading = pose.heading.log();
         double c = Math.cos(heading);
         double s = Math.sin(heading);
         double dxt = botVel.linearVel.x * c - botVel.linearVel.x *s; //chatted transformations maybe scuffed
         double dyt = botVel.linearVel.y * s + botVel.linearVel.y *c;
         vectorXComp = dx- dxt *time;// disp added to robot's vector to goal moving one way -- subtracted from the goal pos the other
         vectorYComp = dy - dyt *time; //may want to replace with a target pose but right now im just moving the vector directly

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
        public void setEnableVelComp(boolean on){
            enableVelComp = on;
        }

        public void resetEncoder(){
        turretMotor.resetEncoder();
        }
    private double clamp(double power,double maxV, double minV) {
    return Math.max(minV,Math.min(maxV,power));
}
}

