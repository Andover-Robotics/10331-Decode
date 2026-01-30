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

@Autonomous(name="Far Blue", group="AA_Autos")
public class FarBlue extends LinearOpMode {
    Bot bot;

    double shootdt =3;


    // inital
    public static Pose2d init = new Pose2d(-60,24,Math.toRadians(0));
    public static Pose2d initRed = new Pose2d(-60,-24,Math.toRadians(0));


    //shooting
    public static Pose2d shoot = new Pose2d(-55,-24,Math.toRadians(-20));//was 20, -30
    public static Vector2d shootPreload = new Vector2d(-58,-24);//was 20,-30


    //intake
    public static Pose2d firstIntake1 = new Pose2d(18,-40,Math.toRadians(-85));//,Math.toRadians(-180)
    public static Vector2d firstIntake2 = new Vector2d(18,-61);//,Math.toRadians(-180)

    public static Vector2d gatePos=new Vector2d(7,-74);


    public static Pose2d secondIntake1 = new Pose2d(-10,-25,Math.toRadians(-85));
    public static Vector2d secondIntake2 = new Vector2d(-10,-53);

    public static Pose2d thirdIntake1 = new Pose2d(-34,-20,Math.toRadians(-90));
    public static Vector2d thirdIntake2 = new Vector2d(-34,-62);
    public ExposureControl exposureControl;
    public GainControl gainControl;

    @Override
    public void runOpMode() throws InterruptedException{
        Bot.instance = null;
        bot = Bot.getInstance(this);
        bot.prepFarAuto(true);
        MecanumDrive drive = Bot.drive;
        drive.localizer.setPose(init);

        Action runAuto = drive.actionBuilderBlue(initRed)
                .afterTime(0.01,bot.intake.actionIntake())
                .afterTime(0.01,bot.actionSpinUp()) //TODO: test dt on pathing here
                .strafeToLinearHeading(shootPreload,Math.toRadians(-15))//preload
                .stopAndAdd(bot.actionShootGate())
                .waitSeconds(shootdt)
                .afterTime(0.01,bot.actionStopShoot())
                .stopAndAdd(new InstantAction(()->bot.intake.stopIntake()))

                .strafeToLinearHeading(bot.pose2Vector(thirdIntake1),thirdIntake1.heading.log())//intake1
                .afterTime(0.01,bot.intake.actionIntake())
                .strafeToLinearHeading(thirdIntake2,Math.toRadians(-85))
//                .afterTime(0.01,new InstantAction(()->bot.intake.stopIntake()))


                //  .afterTime(0.01,bot.intake.actionIntake())
                .afterTime(0.3,bot.actionSpinUp())//TODO: test dt on pathing here
                .strafeToLinearHeading(bot.pose2Vector(shoot),shoot.heading.log())
                .stopAndAdd(bot.actionShootGate())
                .waitSeconds(shootdt)
                .afterTime(0.01,bot.actionStopShoot())

                .strafeToLinearHeading(bot.pose2Vector(secondIntake1),secondIntake1.heading.log())//intake2
//                .afterTime(0.01,bot.intake.actionIntake())
                .strafeToLinearHeading(secondIntake2,Math.toRadians(-85))

                .afterTime(0.01,bot.intake.actionIntake())
                .afterTime(0.4,bot.actionSpinUp()) //TODO: test dt on pathing here
                .strafeToLinearHeading(bot.pose2Vector(shoot),shoot.heading.log())
                .stopAndAdd(bot.actionShootGate())
                .waitSeconds(shootdt)
                .stopAndAdd(bot.actionStopShoot())


//                .setTangent(Math.toRadians(180))
//                .splineToLinearHeading(thirdIntake1, Math.toRadians(-90))//intake2
//                 .afterTime(0.01,bot.intake.actionIntake())
//                .strafeToLinearHeading(thirdIntake2,Math.toRadians(-85))
//
//                .setTangent(Math.toRadians(90)) //shoot 3
//                .afterTime(0.01,bot.intake.actionIntake())
//                .afterTime(1.7,bot.actionSpinUp()) //TODO: test dt on pathing here
//                .splineToLinearHeading(shoot,Math.toRadians(60))
//                .stopAndAdd(bot.actionOpenGate())
//                .waitSeconds(3)
//                .stopAndAdd(bot.actionStopShoot())
//                .waitSeconds(1)
                .build();

        waitForStart();
        Actions.runBlocking(
                new ActionHelper.RaceParallelCommand(
                        bot.actionPeriodic(),
                        runAuto
                ));


    }
}
