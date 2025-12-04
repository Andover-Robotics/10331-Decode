package org.firstinspires.ftc.teamcode.Auto;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Auto.miscRR.ActionHelper;
import org.firstinspires.ftc.teamcode.Auto.miscRR.MecanumDrive;
import org.firstinspires.ftc.teamcode.Teleop.Bot;

@Autonomous(name="Close Red", group="AA_Autos")
public class CloseRed extends LinearOpMode {
    Bot bot;


    // inital
    public static Pose2d initialRedPos = new Pose2d(60,-36,Math.toRadians(0));
    //shooting
    public static Pose2d shoot = new Pose2d(20,-30,Math.toRadians(-10));
    public static Vector2d shootPreload = new Vector2d(20,-30);


    //intake
    public static Pose2d firstIntake1 = new Pose2d(6,-40,Math.toRadians(-50));//,Math.toRadians(-180)
    public static Vector2d firstIntake2 = new Vector2d(30,-72);//,Math.toRadians(-180)

    public static Vector2d secondIntake1 = new Vector2d(-18,-40);
    public static Vector2d secondIntake2 = new Vector2d(6,-72);

    public static Pose2d thirdIntake = new Pose2d(-40,-60,Math.toRadians(-90));



    @Override
    public void runOpMode() throws InterruptedException{
        Bot.instance = null;
        bot = Bot.getInstance(this);
        bot.prepAuto();
        MecanumDrive drive = new MecanumDrive(hardwareMap,initialRedPos);


        Action runAuto = drive.actionBuilder(initialRedPos)
                .afterTime(0.01,bot.intake.actionIntake())
                .strafeToLinearHeading(shootPreload,Math.toRadians(-10))//preload
                .stopAndAdd(bot.actionShoot())
                .waitSeconds(2)
                .stopAndAdd(bot.actionStopShoot())
                .stopAndAdd(new InstantAction(()->bot.intake.stopIntake()))

                .splineToLinearHeading(firstIntake1, Math.toRadians(30))//intake1
                .afterTime(0.01,bot.intake.actionIntake())
                .strafeToLinearHeading(firstIntake2,Math.toRadians(-50))
                .afterTime(0.01,new InstantAction(()->bot.intake.stopIntake()))

                .setTangent(Math.toRadians(90)) //shoot 1
                .afterTime(0.01,bot.intake.actionIntake())
                .splineToLinearHeading(shoot,Math.toRadians(90))
                .stopAndAdd(bot.actionShoot())
                .waitSeconds(2)
                .stopAndAdd(bot.actionStopShoot())
                .waitSeconds(1)


//                .strafeToLinearHeading(secondIntake1, Math.toRadians(-50))//intake2
//                .strafeToLinearHeading(secondIntake2,Math.toRadians(-50))

                        .build();


        waitForStart();
        Actions.runBlocking(
                new ActionHelper.RaceParallelCommand(
                        bot.actionPeriodic(),
                        runAuto
                ));


    }
}
