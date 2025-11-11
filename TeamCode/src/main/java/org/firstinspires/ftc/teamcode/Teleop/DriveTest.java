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
    boolean isFast=false;

    @Override
    public void runOpMode() {

        Bot.instance = null;
        bot.shooter.periodic();
        bot = Bot.getInstance(this);
        gp1 = new GamepadEx(gamepad1);
        gp2 = new GamepadEx(gamepad2);
        bot.intake.closeGate();

        waitForStart();
        while (opModeIsActive() && !isStopRequested()) {
            gp1.readButtons();
            gp2.readButtons();
            drive();
          if (gp2.wasJustPressed(GamepadKeys.Button.A)){
              bot.intake.intake_without_sense(0.3);
          }

            if(gp2.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER)){
                if(!isIntake) {
                    isFast = false;
                    bot.intake.intake_without_sense(0.7);
                    bot.intake.closeGate();
                    isIntake = true;
                }
                else{
                    bot.intake.stopIntake();
                    isIntake=false;
                }
            }
            if(gp2.wasJustPressed(GamepadKeys.Button.DPAD_UP)){
                bot.intake.secondIntake.setPower(0);
            }

            if (gp2.wasJustPressed(GamepadKeys.Button.LEFT_BUMPER)) {
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
                    bot.shooter.setTargetRPM(5000);
                    bot.intake.openGate();
                    bot.intake.secondIntake.setPower(1);

                    isShooting=true;
                }
                else{
                    bot.shooter.setTargetRPM(0);
                    bot.intake.secondIntake.setPower(0);
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