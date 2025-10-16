package org.firstinspires.ftc.teamcode.Teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.arcrobotics.ftclib.geometry.Vector2d;
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
    GamepadEx gp1, gp2;
    private List<Action> runningActions = new ArrayList<>();
    private FtcDashboard dash = FtcDashboard.getInstance();
    public VisionPortal visionPortal;
    private double driveSpeed = 1, driveMultiplier = 1;
    public Bot.BotState state = Bot.BotState.AUTO;

    @Override
    public void runOpMode() {


        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        Bot.instance = null;
        bot = Bot.getInstance(this);
        gp1.readButtons();
        gp2.readButtons();
        //starts finding apriltag
        waitForStart();
        while (opModeIsActive() && !isStopRequested()) {
            TelemetryPacket packet = new TelemetryPacket();
            bot.aprilTag.findAprilTag();


            if (state == Bot.BotState.AUTO) {
                if (gp2.wasJustPressed(GamepadKeys.Button.A)) { // shoot
                    runningActions.add(bot.shoot()); // here for testing purposes, set targetRPM with ftc dash
                    //bot.shoot(); will need when PID tuned and acctually using robot
                }
                if (gp2.isDown(GamepadKeys.Button.RIGHT_BUMPER)) {
                    runningActions.add(bot.intake.actionIntake());
                }
                if (gp2.isDown(GamepadKeys.Button.LEFT_BUMPER)) {
                    runningActions.add(bot.intake.actionReverseIntake());
                }
                drive();


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
                telemetry.addData("Distance from Apriltag", bot.aprilTag.getRange());
                telemetry.addData("Angle offset from Apriltag", bot.aprilTag.getBearing());
                telemetry.addData("Bot yaw from Apriltag", bot.aprilTag.getYaw());
//            telemetry.addData("At Speed?",bot.shooter.atSpeed());
//            telemetry.addData("Break Beam state", bot.actionIntake.getSensorState());
                dash.sendTelemetryPacket(packet);
                //visionPortal.stopStreaming();


            }

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

