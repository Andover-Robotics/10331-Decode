package org.firstinspires.ftc.teamcode.Auto;

import static org.firstinspires.ftc.teamcode.Auto.FarBlue.init;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Auto.miscRR.ActionHelper;
import org.firstinspires.ftc.teamcode.Auto.miscRR.MecanumDrive;
import org.firstinspires.ftc.teamcode.Teleop.Bot;

@Autonomous(name="leave blue")
public class LeavBlue extends LinearOpMode {

    public Bot bot;


    @Override
    public void runOpMode() throws InterruptedException {
        Bot.instance = null;
        bot = Bot.getInstance(this);
        bot.prepFarAuto(false);
        MecanumDrive drive = Bot.drive;
        drive.localizer.recalibrateIMU();
        drive.localizer.setPose(init);
        bot.updatePoses();

        Action runAuto = drive.actionBuilderBlue(FarBlue.initRed)
                .strafeTo(new Vector2d(-55, -35))
                .build();


    while (opModeInInit() && !isStopRequested() && !isStarted()) {
        bot.shooter.periodic();
        bot.turret.periodic();
        telemetry.addData("measured RPM", bot.shooter.getRPM());
        telemetry.addData("Target Degrees", bot.turret.getCurrentTicks());
        telemetry.addData("Target Degrees", bot.turret.getTargetDegrees());
        telemetry.addData("Current Degrees", bot.turret.getCurrentDegrees());
        telemetry.addData("x ", Math.round(drive.localizer.getPose().position.x));
        telemetry.addData("y ", Math.round(drive.localizer.getPose().position.y));
        telemetry.addData("heading ", Math.round(drive.localizer.getPose().heading.toDouble()));
        telemetry.addData("x comp ", bot.turret.vectorXComp);
        telemetry.addData("y comp ", bot.turret.vectorYComp);
        telemetry.addData("turret x ", bot.turret.turretX);
        telemetry.addData("turret y ", bot.turret.turretY);
        telemetry.addData("dx ", bot.turret.dx);
        telemetry.addData("dy ", bot.turret.dy);
        telemetry.addData("ccw rel field ", bot.turret.ccwFieldTarget);
        telemetry.addData("ccw rel field ", Math.toDegrees(Math.atan2(bot.turret.dy, bot.turret.dx)));
        telemetry.addData("goal pose ", Bot.goalPose);


        telemetry.addData("ccw robot target", bot.turret.ccwTargetRelToRobot);
        telemetry.addData("cw target ", bot.turret.cwTarget);

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

