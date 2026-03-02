package org.firstinspires.ftc.teamcode.Teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Auto.Old.CloseBlueOne;
import org.firstinspires.ftc.teamcode.Teleop.Subsystems.Hood;
import org.firstinspires.ftc.teamcode.Teleop.Subsystems.Shooter;
import org.firstinspires.ftc.teamcode.Teleop.Subsystems.Turret;

@Config
@TeleOp
public class Tester extends LinearOpMode {
    public Bot bot;
    private FtcDashboard dash = FtcDashboard.getInstance();
    public static double pos;
    public static double rot;
    GamepadEx gp1;
    public long lastTime=0;
    double driveSpeed = 1;
    double driveMultiplier = 1;




    @Override

    public void runOpMode(){

        Bot.instance = null;
        bot = Bot.getInstance(this);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        Turret.isLocked=true;
        bot.shooter.isPeriodic=false;
        if (!Bot.isRed) Bot.isRed = false;
        bot.updatePoses();
        bot.turret.setEnableVelComp(true);
        gp1 = new GamepadEx(gamepad1);
        Bot.drive.localizer.setPose(new Pose2d(-63,-63,0));

        waitForStart();


        while (opModeIsActive()&& !isStopRequested()) {

            long newTime = System.currentTimeMillis();
            double loopTime = newTime-lastTime;

            TelemetryPacket packet = new TelemetryPacket();

            bot.hood.goToHood(rot);
            bot.turret.periodic();
            //bot.shooter.periodic();
            gp1.readButtons();
            drive();


            if(gp1.wasJustPressed(GamepadKeys.Button.A)){
                Bot.isRed = !Bot.isRed;
                bot.updatePoses();
            }
            if(gp1.wasJustPressed(GamepadKeys.Button.BACK)){
                Bot.drive.localizer.setPose(Bot.resetPose);
            }


//             bot.turret.periodic();
//             bot.shooter.periodic();
//             if(gp1.wasJustPressed(GamepadKeys.Button.A)){
//                 bot.shooter.enableShooter(true);
//                 Shooter.target = RPM;
//             }
//            if(gp1.wasJustPressed(GamepadKeys.Button.B)){
//                bot.shooter.enableShooter(false);
//            }
//            if(gp1.wasJustPressed(GamepadKeys.Button.X)){
//                bot.intake.intake_without_sense(-1);
//
//            }
//            if(gp1.wasJustPressed(GamepadKeys.Button.Y)){
//                bot.intake.intake_without_sense(0);
//
//            }






            telemetry.addData("Target Ticks",bot.turret.getTargetTicks());
            telemetry.addData("Current Ticks",bot.turret.getCurrentTicks());
            telemetry.addData("Target Degrees",bot.turret.getTargetDegrees());
            telemetry.addData("Current Degrees",bot.turret.getCurrentDegrees());
            telemetry.addData("Current Power",bot.turret.getCurrentPower());
            //telemetry.addData("current pos", Bot.drive.localizer.getPose());
            telemetry.addData("current distance ", Turret.distance);
            telemetry.addData("current target RPM ", bot.shooter.getTargetRPM());
            telemetry.addData("current RPM ", bot.shooter.getRPM());
            telemetry.addData("is alliance red?",bot.isRed);
            telemetry.addData("Hood Height",bot.hood.getPos());
            telemetry.addData("Loop Time",loopTime);
            telemetry.addData("Pose",Bot.drive.localizer.getPose());
            telemetry.addData("Goal Pose",Bot.goalPose);



            telemetry.update();
            dash.sendTelemetryPacket(packet);
            lastTime=newTime;

        }

    }
    private void drive() {
        driveSpeed = driveMultiplier - 0.7 * gp1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER);
        driveSpeed = Math.max(0, driveSpeed);

        Vector2d driveVector = new Vector2d(gp1.getLeftX(), -gp1.getLeftY()),
                turnVector = new Vector2d(gp1.getRightX(), 0);
        bot.driveRobotCentric(driveVector.getX() * driveSpeed,
                driveVector.getY() * driveSpeed,
                turnVector.getX() * driveSpeed
        );
    }
}

