package org.firstinspires.ftc.teamcode.Teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Teleop.Subsystems.Hood;
import org.firstinspires.ftc.teamcode.Teleop.Subsystems.Turret;

@Config
@TeleOp
public class Tester extends LinearOpMode {
    public BotTest bot;
    private FtcDashboard dash = FtcDashboard.getInstance();
    public static double pos;




    @Override

    public void runOpMode(){
        BotTest.instance = null;
        bot = BotTest.getInstance(this);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        bot.turret.turretMotor.resetEncoder();
        bot.isRed = false;



        waitForStart();

        while (opModeIsActive()&& !isStopRequested()) {
            TelemetryPacket packet = new TelemetryPacket();
             bot.turret.periodic();




            telemetry.addData("Target Ticks",bot.turret.getTargetTicks());
            telemetry.addData("Current Ticks",bot.turret.getCurrentTicks());
            telemetry.addData("Target Degrees",bot.turret.getTargetDegrees());
            telemetry.addData("Current Degrees",bot.turret.getCurrentDegrees());
            telemetry.addData("Current Power",bot.turret.getCurrentPower());
            telemetry.addData("current pos", BotTest.drive.localizer.getPose());


            telemetry.update();
            dash.sendTelemetryPacket(packet);

        }

    }
}
