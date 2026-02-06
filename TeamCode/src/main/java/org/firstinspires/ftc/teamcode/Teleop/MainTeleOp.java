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
import org.firstinspires.ftc.teamcode.Teleop.Subsystems.Turret;

import java.util.ArrayList;
import java.util.List;


@TeleOp(name="MainTeleop",group = "AA_Main")
public class MainTeleOp extends LinearOpMode {
    Bot bot;
    GamepadEx gp1 , gp2;
    private double driveSpeed = 1, driveMultiplier = 1;
    boolean isIntake=false;
    boolean isShooting=false;
    private List<Action> runningActions = new ArrayList<>();
    private FtcDashboard dash = FtcDashboard.getInstance();
    public final int BLUE=20,RED=24, GPP = 21, PPG = 23, PGP = 22;



    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        Bot.instance = null;

        bot = Bot.getInstance(this);
        gp1 = new GamepadEx(gamepad1);
        gp2 = new GamepadEx(gamepad2);
        bot.prepTeleop();
        Bot.useStoredPose();

        while (opModeInInit() && !isStarted() && !isStopRequested()) {
            TelemetryPacket packet = new TelemetryPacket();
            gp1.readButtons();

            if (gp1.wasJustPressed(GamepadKeys.Button.A)) {
                bot.switchAlliance();
                bot.updatePoses();
            }
            if (!bot.isRed) {
                telemetry.addData("alliance", "Blue");
            } else {
                telemetry.addData("alliance", "Red");
            }
            telemetry.update();
        }



        waitForStart();
        while (opModeIsActive() && !isStopRequested()) {
            TelemetryPacket packet = new TelemetryPacket();
           // bot.handleRecoil();
            bot.shooter.periodic();
            bot.turret.periodic();
            bot.hood.updateHood();
            gp1.readButtons();
            gp2.readButtons();
            drive();




            if(gp1.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER)){
                bot.teleopIntake();
            }
            if (gp1.wasJustPressed(GamepadKeys.Button.LEFT_BUMPER)) {
                bot.intake.reverseIntake();
            }

            if (gp2.wasJustPressed(GamepadKeys.Button.DPAD_RIGHT)){
                bot.hood.incrementHood();
            }
            if (gp2.wasJustPressed(GamepadKeys.Button.DPAD_LEFT)){
                bot.hood.decrementHood();
            }

            if(gp2.wasJustPressed(GamepadKeys.Button.B)){
                bot.intake.openGate();
            }
            if(gp2.wasJustPressed(GamepadKeys.Button.DPAD_UP)){
                bot.teleopShoot();
            }
            if(gp2.wasJustPressed(GamepadKeys.Button.X)){
                bot.resetPose();
            }
            if (gp2.wasJustPressed(GamepadKeys.Button.Y)){
                runningActions.add((bot.actionShootGate()));
                isShooting=true;
            }
//
//            if(gp2.wasJustPressed(GamepadKeys.Button.DPAD_DOWN)){
//                Turret.isLocked = !Turret.isLocked;
//            }

            if(gp1.wasJustPressed(GamepadKeys.Button.BACK)||gp2.wasJustPressed(GamepadKeys.Button.BACK)){
                bot.switchAlliance();
            }
            telemetry.addData("target RPM",bot.shooter.getTargetRPM());
            telemetry.addData("measured RPM",bot.shooter.getRPM());
            telemetry.addData("Target Degrees",bot.turret.getTargetDegrees());
            telemetry.addData("Current Degrees",bot.turret.getCurrentDegrees());
            telemetry.addData("Hood position",bot.hood.hoodServo.getPosition());
            telemetry.addData("At Speed?",bot.shooter.atSpeed());
            telemetry.addData("is alliance red?",bot.isRed);
            telemetry.addData("current distance ", Turret.distance);
            telemetry.addData("goal Pose ", Bot.goalPose);
            telemetry.addData("stored Pose ", Bot.storedPose);

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