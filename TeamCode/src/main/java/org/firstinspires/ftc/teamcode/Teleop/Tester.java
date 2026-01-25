package org.firstinspires.ftc.teamcode.Teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Teleop.Subsystems.Hood;
import org.firstinspires.ftc.teamcode.Teleop.Subsystems.Shooter;
import org.firstinspires.ftc.teamcode.Teleop.Subsystems.Turret;

@Config
@TeleOp
public class Tester extends LinearOpMode {
    public Bot bot;
    private FtcDashboard dash = FtcDashboard.getInstance();
    public static int RPM;
    GamepadEx gp1;




    @Override

    public void runOpMode(){
        Bot.instance = null;
        bot = Bot.getInstance(this);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        Turret.isLocked=false;
        bot.shooter.isPeriodic=false;
        if (!bot.isRed) bot.isRed = true;
        bot.updatePoses();
        //bot.turret.setEnableVelComp(true);
        bot.turret.resetEncoder();
        gp1 = new GamepadEx(gamepad1);
        Bot.drive.localizer.setPose(new Pose2d(60,-58,Math.toRadians(45)));



        waitForStart();


        while (opModeIsActive()&& !isStopRequested()) {
            TelemetryPacket packet = new TelemetryPacket();
            gp1.readButtons();
            bot.hood.goToHood(0.3);
             bot.turret.periodic();
             bot.shooter.periodic();
             if(gp1.wasJustPressed(GamepadKeys.Button.A)){
                 bot.shooter.enableShooter(true);
                 Shooter.target = RPM;
             }
            if(gp1.wasJustPressed(GamepadKeys.Button.B)){
                bot.shooter.enableShooter(false);
            }
            if(gp1.wasJustPressed(GamepadKeys.Button.X)){
                bot.intake.intake_without_sense(-1);

            }
            if(gp1.wasJustPressed(GamepadKeys.Button.Y)){
                bot.intake.intake_without_sense(0);

            }





            telemetry.addData("Target Ticks",bot.turret.getTargetTicks());
            telemetry.addData("Current Ticks",bot.turret.getCurrentTicks());
            telemetry.addData("Target Degrees",bot.turret.getTargetDegrees());
            telemetry.addData("Current Degrees",bot.turret.getCurrentDegrees());
            telemetry.addData("Current Power",bot.turret.getCurrentPower());
            telemetry.addData("current pos", Bot.drive.localizer.getPose());
            telemetry.addData("current distance ", Turret.distance);
            telemetry.addData("current target RPM ", bot.shooter.getTargetRPM());
            telemetry.addData("current RPM ", bot.shooter.getRPM());
            telemetry.addData("is alliance red?",bot.isRed);


            telemetry.update();
            dash.sendTelemetryPacket(packet);

        }

    }
}
