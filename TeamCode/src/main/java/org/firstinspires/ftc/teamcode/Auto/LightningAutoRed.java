package org.firstinspires.ftc.teamcode.Auto;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Auto.miscRR.ActionHelper;
import org.firstinspires.ftc.teamcode.Auto.miscRR.MecanumDrive;
import org.firstinspires.ftc.teamcode.Teleop.Bot;

@Autonomous(name="Lighting Auto Red", group="AA_Autos")
public class LightningAutoRed extends LinearOpMode {
    Bot bot;


    // inital

    private Pose2d init = new Pose2d(60,58,Math.toRadians(45));

    public static Pose2d initialRedPos = new Pose2d(60,-58,Math.toRadians(-45));
    //shooting
    public static Pose2d shoot = new Pose2d(45,-42,Math.toRadians(-55));//was 20, -30
    public static Vector2d shootPreload = new Vector2d(45,-42);//was 20,-30


    //intake
    public static Pose2d firstIntake1 = new Pose2d(16,-46,Math.toRadians(-85));//,Math.toRadians(-180)
    public static Vector2d firstIntake2 = new Vector2d(16,-65);//,Math.toRadians(-180)

    public static Pose2d secondIntake1 = new Pose2d(-8,-46,Math.toRadians(-85));
    public static Vector2d secondIntake2 = new Vector2d(-8,-65);

    public static Vector2d gatePos=new Vector2d(3,-74);

    public static Pose2d thirdIntake = new Pose2d(-40,-60,Math.toRadians(-90));



    @Override
    public void runOpMode() throws InterruptedException{
        Bot.instance = null;
        bot = Bot.getInstance(this);
        bot.prepAuto(24);
        MecanumDrive drive = new MecanumDrive(hardwareMap,initialRedPos);


        Action runAuto = drive.actionBuilderRed(initialRedPos)
                .afterTime(0.01,bot.intake.actionIntake())
                .strafeToLinearHeading(shootPreload,Math.toRadians(-55))//preload
                .stopAndAdd(bot.actionShoot())
                .waitSeconds(2)
                .afterTime(0.01,bot.actionStopShoot())
                .stopAndAdd(new InstantAction(()->bot.intake.stopIntake()))

                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(firstIntake1, Math.toRadians(-60))//intake1
                .afterTime(0.01,bot.intake.actionIntake())
                .strafeToLinearHeading(firstIntake2,Math.toRadians(-85))
                .afterTime(0.01,new InstantAction(()->bot.intake.stopIntake()))

                .strafeToLinearHeading(new Vector2d(16,-50),Math.toRadians(0))
                .strafeToLinearHeading(gatePos,Math.toRadians(15))//gate
                .waitSeconds(2)

                .setTangent(Math.toRadians(90)) //shoot 2
                .afterTime(0.01,bot.intake.actionIntake())
                .splineToLinearHeading(shoot,Math.toRadians(60))
                .stopAndAdd(bot.actionShoot())
                .waitSeconds(2)
                .afterTime(0.01,bot.actionStopShoot())

                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(secondIntake1, Math.toRadians(-10))//intake2
               .afterTime(0.01,bot.intake.actionIntake())
                .strafeToLinearHeading(secondIntake2,Math.toRadians(-85))
                .afterTime(0.01,new InstantAction(()->bot.intake.stopIntake()))


                .setTangent(Math.toRadians(90)) //shoot 3
                .afterTime(0.01,bot.intake.actionIntake())
                .splineToLinearHeading(shoot,Math.toRadians(60))
                .stopAndAdd(bot.actionShoot())
                .waitSeconds(3)
                .stopAndAdd(bot.actionStopShoot())
                .waitSeconds(1)



//                .splineToLinearHeading(shoot,Math.toRadians(90)) //shoot 2
//                .stopAndAdd(bot.actionShoot())
//                .waitSeconds(2)


                .build();


        waitForStart();
        Actions.runBlocking(
                new ActionHelper.RaceParallelCommand(
                        bot.actionPeriodic(),
                        runAuto
                ));


    }
}
