package org.firstinspires.ftc.teamcode.Teleop;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


@TeleOp
public class DriveTest extends LinearOpMode {
    Bot bot;
    GamepadEx gp1 , gp2;
    private double driveSpeed = 1, driveMultiplier = 1;
    boolean isIntake=false;
    boolean isShooting=false;

    @Override
    public void runOpMode() {

        Bot.instance = null;
        bot = Bot.getInstance(this);
        gp1 = new GamepadEx(gamepad1);
        gp2 = new GamepadEx(gamepad2);

        waitForStart();
        while (opModeIsActive() && !isStopRequested()) {
            gp1.readButtons();
            gp2.readButtons();
            drive();

            if(gp2.wasJustPressed(GamepadKeys.Button.A)){
                if(!isIntake) {
                    bot.intake.intake_without_sense();
                    isIntake=true;
                }
                else{
                    bot.intake.stopIntake();
                    isIntake=false;
                }
            }
            if(gp2.wasJustPressed(GamepadKeys.Button.DPAD_DOWN)){
                bot.intake.reverseIntake();
            }
            if(gp2.wasJustPressed(GamepadKeys.Button.X)){
                bot.hood.decrementHood();
            }
            if(gp2.wasJustPressed(GamepadKeys.Button.Y)){
                bot.hood.incrementHood();
            }
            if(gp2.wasJustPressed(GamepadKeys.Button.B)){
                if(!isShooting) {
                    bot.shooter.runShooter(1);
                    isShooting=true;
                }
                else{
                    bot.shooter.runShooter(0);
                    isShooting=false;
                }
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