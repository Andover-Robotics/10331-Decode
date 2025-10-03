package org.firstinspires.ftc.teamcode.Teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@Config
@TeleOp
public class CameraTester extends LinearOpMode {
    public Bot bot;
    private FtcDashboard dash = FtcDashboard.getInstance();



    @Override

    public void runOpMode(){
        Bot.instance = null;
        Bot.getInstance(this);

        waitForStart();

        bot.aprilTag.findAprilTag();

        telemetry.addData("Apriltag ID: ", bot.aprilTag.getId());
        telemetry.addData("Distance from Apriltag",bot.aprilTag.getRange());
        telemetry.addData("Angle offset from Apriltag",bot.aprilTag.getBearing());

    }
}
