package org.firstinspires.ftc.teamcode.Teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@Config
@TeleOp
public class CameraTester extends LinearOpMode {
    public BotTest bot;
    private FtcDashboard dash = FtcDashboard.getInstance();



    @Override

    public void runOpMode(){
        BotTest.instance = null;
        bot = BotTest.getInstance(this);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());


        waitForStart();

        while (opModeIsActive()&& !isStopRequested()) {
            TelemetryPacket packet = new TelemetryPacket();
            bot.aprilTag.findAprilTag();
            bot.shooter.periodic();

            telemetry.addData("target RPM",bot.shooter.getTargetRPM());
            telemetry.addData("Measrued RPM",bot.shooter.getRPM());
            telemetry.addData("motor speed",bot.shooter.getShooterPower());
            telemetry.addData("at speed?",bot.shooter.atSpeed());
            telemetry.addData("Apriltag ID: ", bot.aprilTag.getId());
            telemetry.addData("Distance from Apriltag",bot.aprilTag.getRange());
            telemetry.addData("Angle offset from Apriltag",bot.aprilTag.getBearing());
            telemetry.addData("Bot yaw from Apriltag",bot.aprilTag.getYaw());
            telemetry.update();
            dash.sendTelemetryPacket(packet);

        }

    }
}
