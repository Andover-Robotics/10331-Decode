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
import org.firstinspires.ftc.vision.VisionPortal;

import java.util.concurrent.TimeUnit;

@Autonomous(name="Close Red", group="AA_Autos")
public class CloseRed extends LinearOpMode {
    Bot bot;

    double shootdt =0.5;


    // inital
    public static Pose2d initialRedPos = new Pose2d(60,-58,Math.toRadians(-45));
    //shooting
    public static Pose2d shoot = new Pose2d(35,-35,Math.toRadians(-60));//was 20, -30
    public static Vector2d shootPreload = new Vector2d(42,-42);//was 20,-30


    //intake
    public static Pose2d firstIntake1 = new Pose2d(18,-40,Math.toRadians(-85));//,Math.toRadians(-180)
    public static Vector2d firstIntake2 = new Vector2d(18,-61);//,Math.toRadians(-180)

    public static Vector2d gatePos=new Vector2d(7,-74);


    public static Pose2d secondIntake1 = new Pose2d(-10,-40,Math.toRadians(-85));
    public static Vector2d secondIntake2 = new Vector2d(-10,-61);

    public static Pose2d thirdIntake1 = new Pose2d(-31,-44,Math.toRadians(-90));
    public static Vector2d thirdIntake2 = new Vector2d(-31,-68);
    public ExposureControl exposureControl;
    public GainControl gainControl;

    @Override
    public void runOpMode() throws InterruptedException{
        Bot.instance = null;
        bot = Bot.getInstance(this);
        bot.prepAuto(true);
        MecanumDrive drive = Bot.drive;
        drive.localizer.setPose(initialRedPos);

        Action runAuto = drive.actionBuilderRed(initialRedPos)
                .afterTime(0.01,bot.intake.actionIntake())
                .afterTime(0.01,bot.actionSpinUp()) //TODO: test dt on pathing here
                .strafeToLinearHeading(shootPreload,Math.toRadians(-55))//preload
                .stopAndAdd(bot.actionOpenGate())
                .waitSeconds(shootdt)
                .afterTime(0.01,bot.actionStopShoot())
                .stopAndAdd(new InstantAction(()->bot.intake.stopIntake()))

                .setTangent(Math.toRadians(180))
                .splineToLinearHeading(firstIntake1, Math.toRadians(-90))//intake1
                .afterTime(0.01,bot.intake.actionIntake())
                .strafeToLinearHeading(firstIntake2,Math.toRadians(-85))
//                .afterTime(0.01,new InstantAction(()->bot.intake.stopIntake()))

                .setTangent(Math.toRadians(90))
                .strafeToLinearHeading(gatePos,Math.toRadians(0))
//                .afterTime(0.01,new InstantAction(()->bot.intake.stopIntake()))
                .waitSeconds(1.5)

                .setTangent(Math.toRadians(90)) //shoot 2
                .afterTime(0.01,bot.intake.actionIntake())
                .afterTime(1,bot.actionSpinUp()) //TODO: test dt on pathing here
                .splineToLinearHeading(new Pose2d(shoot.component1().x,shoot.component1().y,Math.toRadians(-55)),Math.toRadians(60))
                .stopAndAdd(bot.actionOpenGate())
                .waitSeconds(shootdt)
                .afterTime(0.01,bot.actionStopShoot())

                .setTangent(Math.toRadians(180))
                .splineToLinearHeading(secondIntake1, Math.toRadians(-90))//intake2
//                .afterTime(0.01,bot.intake.actionIntake())
                .strafeToLinearHeading(secondIntake2,Math.toRadians(-85))

                .setTangent(Math.toRadians(90)) //shoot 3
                .afterTime(0.01,bot.intake.actionIntake())
                .afterTime(1.5,bot.actionSpinUp()) //TODO: test dt on pathing here
                .splineToLinearHeading(shoot,Math.toRadians(60))
                .stopAndAdd(bot.actionOpenGate())
                .waitSeconds(shootdt)
                .stopAndAdd(bot.actionStopShoot())


                .setTangent(Math.toRadians(180))
                .splineToLinearHeading(thirdIntake1, Math.toRadians(-90))//intake2
//                .afterTime(0.01,bot.intake.actionIntake())
                .strafeToLinearHeading(thirdIntake2,Math.toRadians(-85))

                .setTangent(Math.toRadians(90)) //shoot 3
                .afterTime(0.01,bot.intake.actionIntake())
                .afterTime(1.7,bot.actionSpinUp()) //TODO: test dt on pathing here
                .splineToLinearHeading(shoot,Math.toRadians(60))
                .stopAndAdd(bot.actionOpenGate())
                .waitSeconds(3)
                .stopAndAdd(bot.actionStopShoot())
                .waitSeconds(1)
                .build();

        waitForStart();
        Actions.runBlocking(
                new ActionHelper.RaceParallelCommand(
                        bot.actionPeriodic(),
                        runAuto
                ));


    }
}
