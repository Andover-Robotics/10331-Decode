package org.firstinspires.ftc.teamcode.Teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Teleop.Subsystems.Hood;

import java.util.ArrayList;
import java.util.List;


@TeleOp
public class MainTeleOp extends LinearOpMode {
    Bot bot;
    GamepadEx gp1 , gp2;
    private double driveSpeed = 1, driveMultiplier = 1;
    boolean isIntake=false;
    boolean isShooting=false;
    boolean isSecondIntaking=false, isReverseSec=false;
    private List<Action> runningActions = new ArrayList<>();
    private FtcDashboard dash = FtcDashboard.getInstance();
    public final int BLUE=20,RED=24, GPP = 21, PPG = 23, PGP = 22;


    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        Bot.instance = null;

        bot = Bot.getInstance(this);
        gp1 = new GamepadEx(gamepad1);
        gp2 = new GamepadEx(gamepad2);
        bot.intake.closeGate();
        bot.hood.hoodServo.setPosition(0.6);
        Hood.outtakePos=0.3;

        while (!isStarted()) {
            TelemetryPacket packet = new TelemetryPacket();
            gp1.readButtons();
            if (gp1.wasJustPressed(GamepadKeys.Button.A)) {
                bot.switchAlliance();
            }
            if (bot.aprilTag.targetAllianceId == 20) {
                telemetry.addData("alliance", "Blue");
            } else {
                telemetry.addData("alliance", "Red");
            }
            telemetry.update();
        }



        waitForStart();
        while (opModeIsActive() && !isStopRequested()) {
            TelemetryPacket packet = new TelemetryPacket();
            bot.aprilTag.findAprilTag();
            bot.shooter.periodic();


            gp1.readButtons();
            gp2.readButtons();
            drive();


            if(gp2.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER)){
                if(!isIntake) {
                    bot.intake.intake_without_sense(0.7);
                    isIntake = true;
                }
                else{
                    bot.intake.stopIntake();
                    isIntake=false;
                }
            }

            if (gp2.wasJustPressed(GamepadKeys.Button.LEFT_BUMPER)) {
                bot.intake.reverseIntake();
            }

            if (gp2.wasJustPressed(GamepadKeys.Button.DPAD_UP)){
                if (bot.hood.hoodServo.getPosition()<=0.73) {
                    Hood.outtakePos += 0.05;
                }
                bot.hood.goToHood(Hood.outtakePos);
            }
            if (gp2.wasJustPressed(GamepadKeys.Button.DPAD_DOWN)){
                Hood.outtakePos-=0.05;
                bot.hood.goToHood(Hood.outtakePos);

            }
            if(gp2.wasJustPressed(GamepadKeys.Button.B)){
                if(!isShooting) {
                    runningActions.add(bot.actionShoot());
                    isShooting=true;
                }
                else{
                    bot.shooter.setTargetRPM(0);
                    bot.intake.closeGate();
                    isShooting=false;
                }
            }
            if (gp2.wasJustPressed(GamepadKeys.Button.Y)){
                runningActions.add((bot.actionShootGate()));
                isShooting=true;
            }

            if(gp1.wasJustPressed(GamepadKeys.Button.BACK)||gp2.wasJustPressed(GamepadKeys.Button.BACK)){
                bot.switchAlliance();
            }
            telemetry.addData("Apriltag ID: ", bot.aprilTag.getId());
            telemetry.addData("Distance from Apriltag", bot.aprilTag.getRange());
            telemetry.addData("Angle offset from Apriltag", bot.aprilTag.getBearing());
            telemetry.addData("Bot yaw from Apriltag", bot.aprilTag.getYaw());
            telemetry.addData("target RPM",bot.shooter.getTargetRPM());
            telemetry.addData("measured RPM",bot.shooter.getRPM());

            telemetry.addData("Hood position",bot.hood.hoodServo.getPosition());
            telemetry.addData("distance:",bot.aprilTag.calcAccurateDis());
            telemetry.addData("At Speed?",bot.shooter.atSpeed());
            if (bot.aprilTag.targetAllianceId == 20) {
                telemetry.addData("alliance", "Blue");
            } else {
                telemetry.addData("alliance", "Red");
            }
            telemetry.update();

            List<Action> newActions = new ArrayList<>();
            for (Action action : runningActions) {
                action.preview(packet.fieldOverlay());
                if (action.run(packet)) {
                    newActions.add(action);
                }
            }
            runningActions = newActions;

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