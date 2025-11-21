package org.firstinspires.ftc.teamcode.Teleop.Subsystems;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.Teleop.Bot;

import java.util.List;

public class AutoAlign extends LinearOpMode {

    Bot bot;
    double bearing;


    @Override
    public void runOpMode() throws InterruptedException {

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        Bot.instance = null;
        bot = Bot.getInstance(this);
        bearing = bot.returnIMU();

        List<MotorEx> motorList = bot.returnMotors();
        MotorEx fl = motorList.get(0);
        MotorEx bl = motorList.get(1);
        MotorEx br = motorList.get(2);
        MotorEx fr = motorList.get(3);

        waitForStart();
        while (opModeIsActive() && !isStopRequested()) {
            TelemetryPacket packet = new TelemetryPacket();
            bot.aprilTag.findAprilTag();

        }
    }
}
