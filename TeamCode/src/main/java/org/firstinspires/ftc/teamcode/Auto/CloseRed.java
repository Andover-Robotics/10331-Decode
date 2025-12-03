package org.firstinspires.ftc.teamcode.Auto;

import com.acmerobotics.roadrunner.Action;
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
    public static Pose2d initialRedPos = new Pose2d(50,50,Math.toRadians(-135));
    //shooting
    public static Pose2d shoot = new Pose2d(12,12,Math.toRadians(-135));
    public static Vector2d shootPreload = new Vector2d(12,12);


    //intake
    public static Vector2d firstIntake = new Vector2d(60,12);//,Math.toRadians(-180)
    public static Pose2d secondIntake = new Pose2d(60,-12,Math.toRadians(-180));
    public static Pose2d thirdIntake = new Pose2d(60,-36,Math.toRadians(-180));



    @Override
    public void runOpMode() throws InterruptedException{
        Bot.instance = null;
        bot = Bot.getInstance(this);
        MecanumDrive drive = new MecanumDrive(hardwareMap,initialRedPos);


        Action runAuto = drive.actionBuilder(initialRedPos)
                .strafeToConstantHeading(shootPreload)
                .waitSeconds(3)
                .strafeToLinearHeading(firstIntake,Math.toRadians(-180))
                .strafeToLinearHeading(shootPreload,Math.toRadians(-135))
                        .build();


        waitForStart();
        Actions.runBlocking(
                new ActionHelper.RaceParallelCommand(
                        bot.actionPeriodic(),
                        runAuto
                ));


    }
}
