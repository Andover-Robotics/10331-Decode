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

import java.util.ArrayList;

@Autonomous(name="Close Blue", group="AA_Autos")
public class CloseBlue extends LinearOpMode {
    Bot bot;

    // inital

    double shootdt = 1.7;
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

    public static Pose2d thirdIntake1 = new Pose2d(-34,-40,Math.toRadians(-90));
    public static Vector2d thirdIntake2 = new Vector2d(-34,-62);
    public ExposureControl exposureControl;
    public GainControl gainControl;
    public ArrayList<Action> actionsToRun;




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

        Action preLoad = drive.actionBuilderBlue(initialRedPos) // How we mirror the cords & call all methods
                // make sure intake is constant running
                // shoot preloaded artis (2-3 secs)
                .afterTime(0.01,bot.intake.actionIntakeClose())
                .afterTime(0.01, bot.actionSpinUp())
                .strafeToSplineHeading(shootPreload, Math.toRadians(-50))
                // bot is now in pos , not waiting .3 secs cuz turret will compensate
                .stopAndAdd(bot.actionOpenGate())






                .build();

        Action frontRow = drive.actionBuilderBlue(initialRedPos)
                // go to front pos
                .build();

        Action middleRow = drive.actionBuilderBlue(initialRedPos)
                // middle pos
                .build();

        Action backRow = drive.actionBuilderBlue(initialRedPos)
                // back pos
                .build():

        /*Action gateCycle = drive.actionBuilderBlue(initialRedPos)
                // intake & shoot middle row (5 secs)
                // pos bot so it stays at gate so its open, (with wedges)
                // for loop (3 times (4 if 21)
                int i;
                for(i = 0 ; i < 3 ; i++) {
                    // cycle through (intake shoot, x3/x4)

                }
                .build();*/


        Action eighteenAris = drive.actionBuilderBlue(initialRedPos)
                // intake & shoot middle row (5 secs)
                // for loop (3 times (4 if 21)



                .build();


        waitForStart();
        // running the paths
        Actions.runBlocking(
                new ActionHelper.RaceParallelCommand(
                        bot.actionPeriodic()
                        // change the action/actions running based off of what we want to run
                ));





    }


}
