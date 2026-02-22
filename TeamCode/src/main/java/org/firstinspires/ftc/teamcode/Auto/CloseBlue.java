package org.firstinspires.ftc.teamcode.Auto;

import com.acmerobotics.roadrunner.Action;
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
   private static double gatedt=4;
   public static double gateCycles=0;

    private Pose2d init = new Pose2d(60,48,Math.toRadians(0));
    public static Pose2d initialRedPos = new Pose2d(60,-48,Math.toRadians(0));
    //shooting
    public static Pose2d shoot = new Pose2d(33,-30,Math.toRadians(-50));//was 20, -30
    public static Vector2d shootPreload = new Vector2d(33,-30);//was 20,-30

    //intake
    public static Pose2d firstIntake1 = new Pose2d(13,-37,Math.toRadians(-85));//,Math.toRadians(-180)
    public static Vector2d firstIntake2 = new Vector2d(13,-59);//,Math.toRadians(-180)

    public static Vector2d gatePos=new Vector2d(3,-68);


    public static Pose2d secondIntake1 = new Pose2d(-13,-35,Math.toRadians(-90));
    public static Vector2d secondIntake2 = new Vector2d(-13,-65);

    public static Pose2d gateCyclePos = new Pose2d(7,-75,Math.toRadians(-45));

    public static Pose2d thirdIntake1 = new Pose2d(-34,-40,Math.toRadians(-90));
    public static Vector2d thirdIntake2 = new Vector2d(-34,-62);
    Action eighteenAris;
    public ExposureControl exposureControl;
    public GainControl gainControl;

    private TrajectoryActionBuilder builder;
    public ArrayList<Action> paths = new ArrayList<>();

    @Override
    public void runOpMode() throws InterruptedException{
        Bot.instance = null;
        bot = Bot.getInstance(this);
        bot.prepAuto(false);
        MecanumDrive drive = Bot.drive;

        drive.localizer.setPose(init);

        // notes for nicole: vector is just the endgoal, pos is the angle of the bot AND the position
        // straf = short distances better, ex: going straight lines
        // spline = creating curve to hit the destination faster

        Action preload = drive.actionBuilderBlue(initialRedPos) // How we mirror the cords & call all methods
                // make sure intake is constant running
                // shoot preloaded artis (2-3 secs)
                .afterTime(0.01,bot.intake.actionIntakeClose())
                .afterTime(0.01, bot.actionSpinUp())
                .strafeToSplineHeading(shootPreload, Math.toRadians(-50))
                // bot is now in pos , not waiting .3 secs cuz turret will compensate
                .stopAndAdd(bot.actionOpenGate())
                .waitSeconds(shootdt)
                .build();

        Action frontRow = drive.actionBuilderBlue(shoot)
                .setReversed(true)
                .splineToSplineHeading(firstIntake1,Math.toRadians(90))
                .strafeToSplineHeading(firstIntake2,Math.toRadians(-90))
                .setReversed(true)
                .afterTime(0.2, bot.actionSpinUp())
                .splineTo(bot.pose2Vector(shoot),Math.toRadians(90))
                .stopAndAdd(bot.actionOpenGate())
                .waitSeconds(shootdt)
                .build();

        Action middleRow = drive.actionBuilderBlue(shoot)
                .setReversed(true)
                .splineToSplineHeading(secondIntake1,Math.toRadians(90))
                .strafeToSplineHeading(secondIntake2,Math.toRadians(-90))
                .setReversed(true)
                .afterTime(0.2, bot.actionSpinUp())
                .splineTo(bot.pose2Vector(shoot),Math.toRadians(90))
                .stopAndAdd(bot.actionOpenGate())
                .waitSeconds(shootdt)

                .build();



//       Turn into an array list because for some reason it keeps creating the action over and over again
        Action eighteenAris = drive.actionBuilderBlue(shoot)
                .setReversed(true)
                .splineToSplineHeading(gateCyclePos,Math.toRadians(-150))
                .stopAndAdd(bot.intake.actionIntakeClose())
                .waitSeconds(gatedt)
                .afterTime(0.2,bot.actionSpinUp())
                .splineTo(bot.pose2Vector(shoot),Math.toRadians(90))
                .stopAndAdd(bot.actionOpenGate())
                .waitSeconds(shootdt)
                .build();


        waitForStart();
        // running the paths
        Actions.runBlocking(
                new ActionHelper.RaceParallelCommand(
                        bot.actionPeriodic(),
                        new SequentialAction(
                                preload,
                                middleRow,
                                eighteenAris,
                                eighteenAris,
                                eighteenAris,
                                frontRow
                        // change the action/actions running based off of what we want to run
                )));

    }
}
