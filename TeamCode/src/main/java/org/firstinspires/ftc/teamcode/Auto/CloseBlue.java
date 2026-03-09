package org.firstinspires.ftc.teamcode.Auto;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Trajectory;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.teamcode.Auto.miscRR.ActionHelper;
import org.firstinspires.ftc.teamcode.Auto.miscRR.MecanumDrive;
import org.firstinspires.ftc.teamcode.Teleop.Bot;

import java.util.ArrayList;
@Autonomous(name="Close Blue", group="AA_Autos")
public class CloseBlue extends LinearOpMode {
    Bot bot;

    // inital

   private static double shootdt = 1.7;
   private static double gatedt=2;
   public static double gateCycles=0;

    private Pose2d init = new Pose2d(60,48,Math.toRadians(0));
    public static Pose2d initialRedPos = new Pose2d(60,-48,Math.toRadians(0));
    //shooting
    public static Pose2d shoot = new Pose2d(33,-30,Math.toRadians(0));//was 20, -30
    public static Vector2d shootPreload = new Vector2d(33,-30);//was 20,-30

    //intake
    public static Pose2d firstIntake1 = new Pose2d(9,-36,Math.toRadians(-85));//,Math.toRadians(-180)
    public static Vector2d firstIntake2 = new Vector2d(9,-52);//,Math.toRadians(-180)

    public static Vector2d gatePos=new Vector2d(3,-68);


    public static Pose2d secondIntake1 = new Pose2d(-17,-35,Math.toRadians(-90));
    public static Vector2d secondIntake2 = new Vector2d(-17,-62);

    public static Pose2d gateCyclePos0 = new Pose2d(-17,-40,Math.toRadians(-85));
    public static Pose2d gateCyclePos1 = new Pose2d(-17,-68,Math.toRadians(-85));


    public static Pose2d thirdIntake1 = new Pose2d(-34,-40,Math.toRadians(-90));
    public static Vector2d thirdIntake2 = new Vector2d(-34,-62);


    @Override
    public void runOpMode() throws InterruptedException{
        Bot.instance = null;
        bot = Bot.getInstance(this);
        bot.prepAuto(false);
        MecanumDrive drive = Bot.drive;
        drive.localizer.recalibrateIMU();
        drive.localizer.setPose(init);
        bot.updatePoses();

        // notes for nicole: vector is just the endgoal, pos is the angle of the bot AND the position
        // straf = short distances better, ex: going straight lines
        // spline = creating curve to hit the destination faster


        Action runAuto = drive.actionBuilderBlue(initialRedPos)
                .afterTime(0.01,bot.intake.actionIntakeClose())
                .afterTime(0.01,bot.actionSpinUp()) //TODO: test dt on pathing here
                .strafeTo(bot.pose2Vector(shoot))//preload
                .waitSeconds(0.3)
                .stopAndAdd(bot.actionOpenGate())
                .waitSeconds(shootdt)
                .afterTime(0.01,bot.actionStopShoot())
                .stopAndAdd(new InstantAction(()->bot.intake.stopIntake()))

                .setTangent(Math.toRadians(190))
                .splineToSplineHeading(firstIntake1, Math.toRadians(-90))//intake1
                .afterTime(0.01,bot.intake.actionIntakeClose())
                .strafeToSplineHeading(firstIntake2,Math.toRadians(-85))
//                .afterTime(0.01,new InstantAction(()->bot.intake.stopIntake()))

//                .setTangent(Math.toRadians(90))
//                .strafeToLinearHeading(gatePos,Math.toRadians(0))
//                .waitSeconds(1.5)

                .setTangent(Math.toRadians(90)) //shoot 2
                .afterTime(0.01,bot.intake.actionIntakeClose())
                .afterTime(0.2,bot.actionSpinUp()) //TODO: test dt on pathing here
                .splineToSplineHeading(new Pose2d(shoot.component1().x,shoot.component1().y,Math.toRadians(-55)),Math.toRadians(60))
                .stopAndAdd(bot.actionOpenGate())
                .waitSeconds(shootdt)
                .afterTime(0.01,bot.actionStopShoot())

                .setReversed(true)
                .splineToSplineHeading(secondIntake1, Math.toRadians(-90))//intake2
//               .afterTime(0.01,bot.intake.actionIntakeClose())
                .strafeToSplineHeading(secondIntake2,Math.toRadians(-95))

                .setTangent(Math.toRadians(90)) //shoot 3
                .afterTime(0.01,bot.intake.actionIntakeClose())
                .afterTime(0.3,bot.actionSpinUp()) //TODO: test dt on pathing here
                .splineTo(bot.pose2Vector(shoot),Math.toRadians(60))
                .stopAndAdd(bot.actionOpenGate())
                .waitSeconds(shootdt)
                .stopAndAdd(bot.actionStopShoot())

                .setReversed(true)
                .splineToSplineHeading(gateCyclePos0,Math.toRadians(-90))
                .strafeToSplineHeading(bot.pose2Vector(gateCyclePos1),Math.toRadians(-55))
                .stopAndAdd(bot.intake.actionIntakeClose())
                .waitSeconds(gatedt)
                .afterTime(0.2,bot.actionSpinUp())
                .setReversed(true)
                .splineTo(bot.pose2Vector(shoot),Math.toRadians(-40))
                .stopAndAdd(bot.actionOpenGate())
                .waitSeconds(shootdt)
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

//


        waitForStart();
        // running the paths
        Actions.runBlocking(
                new ActionHelper.RaceParallelCommand(
                        bot.actionPeriodic(),
                        runAuto
                ));
    }
}
