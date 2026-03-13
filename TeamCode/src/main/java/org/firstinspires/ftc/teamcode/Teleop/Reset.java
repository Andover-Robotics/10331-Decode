package org.firstinspires.ftc.teamcode.Teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import java.util.Map.Entry;

@TeleOp(name ="Reset Everything",group = "AA_Main")
//used to reset all motor encoders and other things that need to be reset without turning off robor
public class Reset extends LinearOpMode {

    Bot bot;
    @Override
    public void runOpMode(){
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
                Bot.instance = null;

        bot = Bot.getInstance(this);

        Bot.drive.localizer.recalibrateIMU();


        waitForStart();
        bot.shooter.reset();
        bot.turret.resetEncoder();
        }





    }

