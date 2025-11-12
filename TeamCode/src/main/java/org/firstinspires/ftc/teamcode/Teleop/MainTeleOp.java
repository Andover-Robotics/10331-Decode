package org.firstinspires.ftc.teamcode.Teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.arcrobotics.ftclib.geometry.Vector2d;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Teleop.Subsystems.Hood;

import java.util.ArrayList;
import java.util.List;
@TeleOp
@Config

public class MainTeleOp extends LinearOpMode {
    Bot bot;
    GamepadEx gp1 , gp2;
    private List<Action> runningActions = new ArrayList<>();
    private FtcDashboard dash = FtcDashboard.getInstance();
    private double driveSpeed = 1, driveMultiplier = 1;
    public Bot.BotState state = Bot.BotState.AUTO;
    private boolean isIntaking, isShooting;
    public final int BLUE=20,RED=24, GPP = 21, PPG = 23, PGP = 22;

    @Override
    public void runOpMode() {

        while (opModeInInit()){
            gp1.readButtons();
            if(gp1.wasJustPressed(GamepadKeys.Button.A)){
                bot.aprilTag.targetAllianceId = RED;
            }
            if (gp1.wasJustPressed(GamepadKeys.Button.B)){
                bot.aprilTag.targetAllianceId = BLUE;
            }
        }
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        Bot.instance = null;
        bot = Bot.getInstance(this);
        gp1 = new GamepadEx(gamepad1);
        bot.hood.goToHood(0.3);


         //TODO: allow alliance selection in init

        //starts finding apriltag
        waitForStart();
        while (opModeIsActive() && !isStopRequested()) {
            TelemetryPacket packet = new TelemetryPacket();
            gp1.readButtons();
            bot.aprilTag.findAprilTag();
            bot.shooter.periodic();




                if (gp1.wasJustPressed(GamepadKeys.Button.A)) {
                    runningActions.add(bot.actionShoot());
                }
                 if (gp1.wasJustPressed(GamepadKeys.Button.B)){
                    bot.shooter.setTargetRPM(0);
                }

                if (gp1.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER)) {
                    bot.intake.intake_without_sense();
                }
                if (gp1.wasJustPressed(GamepadKeys.Button.Y)){
                    bot.intake.stopIntake();

                }
//                if (gp1.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER)){
//                    runningActions.add(bot.intake.openGate(1.7));
//                }

                if (gp1.wasJustPressed(GamepadKeys.Button.DPAD_UP)){
                    if (bot.hood.hoodServo.getPosition()<=0.73) {
                        Hood.outtakePos += 0.05;
                    }
                    bot.hood.goToHood(Hood.outtakePos);
                }
                if (gp1.wasJustPressed(GamepadKeys.Button.DPAD_DOWN)){
                    Hood.outtakePos-=0.05;
                bot.hood.goToHood(Hood.outtakePos);

            }



               //drive();


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
                telemetry.addData("target RPM",bot.shooter.getTargetRPM());
                telemetry.addData("Hood position",bot.hood.hoodServo.getPosition());
                telemetry.addData("distance:",bot.aprilTag.calcAccurateDis());
                telemetry.addData("At Speed?",bot.shooter.atSpeed());
//            telemetry.addData("Break Beam state", bot.actionIntake.getSensorState());
                dash.sendTelemetryPacket(packet);
                //visionPortal.stopStreaming();




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

