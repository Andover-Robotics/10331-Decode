package org.firstinspires.ftc.teamcode.Teleop;

import android.graphics.Camera;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.vision.VisionPortal;

import java.util.ArrayList;
import java.util.List;
@TeleOp

public class MainTeleOp extends LinearOpMode {
    Bot bot;
    GamepadEx gp1,gp2;
    private List<Action> runningActions = new ArrayList<>();
    private FtcDashboard dash = FtcDashboard.getInstance();
    public VisionPortal visionPortal;
    public Bot.BotState state = Bot.BotState.AUTO;

    @Override
    public void runOpMode(){


        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        Bot.instance = null;
        Bot.getInstance(this);
        gp1.readButtons();
        gp2.readButtons();
        //starts finding apriltags
        bot.aprilTag.findAprilTag();
        waitForStart();
        while(opModeIsActive() && !isStopRequested()){
            TelemetryPacket packet = new TelemetryPacket();
            bot.aprilTag.visionPortal.resumeStreaming();

            if (state == Bot.BotState.AUTO) {
                if (gp1.wasJustPressed(GamepadKeys.Button.A)) {
                    bot.shooter.periodic();
                }
            }

            else {
                // idk
            }

            //teleop code here





            //updates the readings sent in telemetry packet
            telemetry.update();
            // update runningActions arraylist
            List<Action> newActions = new ArrayList<>();
            for (Action action : runningActions) {
                action.preview(packet.fieldOverlay());
                if (action.run(packet)) {
                    newActions.add(action);
                }
            }
            runningActions = newActions;


            // send telemetry to dashboard
            telemetry.addData("Apriltag ID: ", bot.aprilTag.getId());
            telemetry.addData("Distance from Apriltag",bot.aprilTag.getRange());
            telemetry.addData("Angle offset from Apriltag",bot.aprilTag.getBearing());
            telemetry.addData("At Speed?",bot.shooter.atSpeed());
            telemetry.addData("Break Beam state", bot.intake.getSensorState());
            dash.sendTelemetryPacket(packet);
            //visionPortal.stopStreaming();


        }

    }
}
