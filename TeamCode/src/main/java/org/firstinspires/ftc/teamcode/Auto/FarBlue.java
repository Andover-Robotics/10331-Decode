package org.firstinspires.ftc.teamcode.Auto;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.teamcode.Auto.miscRR.ActionHelper;
import org.firstinspires.ftc.teamcode.Auto.miscRR.MecanumDrive;
import org.firstinspires.ftc.teamcode.Teleop.Bot;

@Autonomous(name="Far Blue", group="AA_Autos")
public class FarBlue extends LinearOpMode {
    Bot bot;

    double shootdt =0.5;


    // inital
    public static Pose2d init = new Pose2d(-60,24,Math.toRadians(0));
    public static Pose2d initRed = new Pose2d(-60,-24,Math.toRadians(0));


    //shooting
    public static Pose2d shoot = new Pose2d(-55,-19,Math.toRadians(-25));//was 20, -30
    public static Vector2d shootPreload = new Vector2d(-55,-19);//was 20,-30


    //intake
    public static Pose2d secretTunnel1 = new Pose2d(18,-40,Math.toRadians(-85));//,Math.toRadians(-180)
    public static Vector2d firstIntake2 = new Vector2d(18,-61);//,Math.toRadians(-180)

    public static Vector2d gatePos=new Vector2d(7,-74);


    public static Pose2d hpIntake1 = new Pose2d(-55,-35,Math.toRadians(-110));
    public static Vector2d hpintake2 = new Vector2d(-72,-63);

    public static Pose2d thirdIntake1 = new Pose2d(-42,-35,Math.toRadians(-90));
    public static Vector2d thirdIntake2 = new Vector2d(-42,-58);
    public ExposureControl exposureControl;
    public GainControl gainControl;

    @Override
    public void runOpMode() throws InterruptedException{
        Bot.instance = null;
        bot = Bot.getInstance(this);
        bot.prepFarAuto(false);
        MecanumDrive drive = Bot.drive;
        drive.localizer.recalibrateIMU();
        drive.localizer.setPose(init);
        bot.updatePoses();

        Action runAuto = drive.actionBuilderBlue(initRed)
                .afterTime(0.01,bot.intake.actionIntakeFar())
                .afterTime(0.01,bot.actionSpinUp()) //TODO: test dt on pathing here
                .waitSeconds(1)
                .stopAndAdd(bot.actionShootGate())
                .afterTime(0.01,bot.actionStopShoot())
                .stopAndAdd(new InstantAction(()->bot.intake.stopIntake()))


                .splineToSplineHeading(thirdIntake1,Math.toRadians(-90))//intake1
                .afterTime(0.01,bot.intake.actionIntakeFar())
                .afterTime(0.01,bot.intake.actionIntakeFar())
                .strafeToSplineHeading(thirdIntake2,Math.toRadians(-85))
//                .afterTime(0.01,new InstantAction(()->bot.intake.stopIntake()))


                //  .afterTime(0.01,bot.intake.actionIntakeClose())
                .setReversed(true)
                .afterTime(0.3,bot.actionSpinUp())//TODO: test dt on pathing here
                .splineToSplineHeading(shoot,Math.toRadians(100))
                .waitSeconds(1)
                .stopAndAdd(bot.actionShootGate())
                .afterTime(0.01,bot.actionStopShoot())

                .splineToSplineHeading(hpIntake1,Math.toRadians(-90))//intake2
//                .afterTime(0.01,bot.intake.actionIntakeClose())
                .strafeToSplineHeading(hpintake2,Math.toRadians(-85))

                .setReversed(true)
                .afterTime(0.01,bot.intake.actionIntakeFar())
                .afterTime(0.4,bot.actionSpinUp()) //TODO: test dt on pathing here
                .splineToSplineHeading(shoot,Math.toRadians(100))
                .waitSeconds(1)
                .stopAndAdd(bot.actionShootGate())
                .stopAndAdd(bot.actionStopShoot())

                .afterTime(0.01, bot.intake.actionIntakeFar())
                //.splineToSplineHeading()





//                .setTangent(Math.toRadians(180))
//                .splineToLinearHeading(thirdIntake1, Math.toRadians(-90))//intake2
//                 .afterTime(0.01,bot.intake.actionIntakeClose())
//                .strafeToLinearHeading(thirdIntake2,Math.toRadians(-85))
//
//                .setTangent(Math.toRadians(90)) //shoot 3
//                .afterTime(0.01,bot.intake.actionIntakeClose())
//                .afterTime(1.7,bot.actionSpinUp()) //TODO: test dt on pathing here
//                .splineToLinearHeading(shoot,Math.toRadians(60))
//                .stopAndAdd(bot.actionOpenGate())
//                .waitSeconds(3)
//                .stopAndAdd(bot.actionStopShoot())
//                .waitSeconds(1)
                .build();

        while (opModeInInit() && !isStopRequested() && !isStarted()) {
            bot.shooter.periodic();
            bot.turret.periodic();
            telemetry.addData("measured RPM",bot.shooter.getRPM());
            telemetry.addData("Target Degrees",bot.turret.getCurrentTicks());
            telemetry.addData("Target Degrees",bot.turret.getTargetDegrees());
            telemetry.addData("Current Degrees",bot.turret.getCurrentDegrees());
            telemetry.addData("x ", Math.round(drive.localizer.getPose().position.x));
            telemetry.addData("y ", Math.round(drive.localizer.getPose().position.y));
            telemetry.addData("heading ", Math.round(drive.localizer.getPose().heading.toDouble()));
            telemetry.addData("x comp ",bot.turret.vectorXComp);
            telemetry.addData("y comp ",bot.turret.vectorYComp);
            telemetry.addData("turret x ",bot.turret.turretX);
            telemetry.addData("turret y ",bot.turret.turretY);
            telemetry.addData("dx ",bot.turret.dx);
            telemetry.addData("dy ",bot.turret.dy);
            telemetry.addData("ccw rel field ",bot.turret.ccwFieldTarget);
            telemetry.addData("ccw rel field ",Math.toDegrees(Math.atan2(bot.turret.dy,bot.turret.dx)));
            telemetry.addData("goal pose ",Bot.goalPose);



            telemetry.addData("ccw robot target" ,bot.turret.ccwTargetRelToRobot);
            telemetry.addData("cw target ",bot.turret.cwTarget);

            telemetry.update();

        }

        waitForStart();
        Actions.runBlocking(
                new ActionHelper.RaceParallelCommand(
                        bot.actionPeriodic(),
                        runAuto
                ));


    }
}
