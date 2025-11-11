package org.firstinspires.ftc.teamcode.Auto;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Auto.miscRR.ActionHelper;
import org.firstinspires.ftc.teamcode.Auto.miscRR.MecanumDrive;
import org.firstinspires.ftc.teamcode.Teleop.Bot;
@Config
@Autonomous(name = "Solo Auto", group = "Autonomous")
public class SoloAuto extends LinearOpMode {
    Bot bot;
    private GamepadEx gp1;
    private final int BLUE = 20, RED = 24, GPP = 21, PPG = 23, PGP = 22;
    private AutoPos chosenAuto;
    public enum AutoPos{
        CLOSE_BLUE(
                new Pose2d(60, -8, Math.toRadians(-180)), //starting pos
                new Pose2d(36, 8, Math.toRadians(90)), //firstPreIntake pos
                new Pose2d(36, -52, Math.toRadians(-90)), //firstIntakePos
                new Pose2d(12, -8, Math.toRadians(-90)), //secondPreIntake pos
                new Pose2d(12, -52, Math.toRadians(-90)), //secondIntake pos
                new Pose2d(60, -8, Math.toRadians(0)) //shooting pos
        ),
        CLOSE_RED(
                new Pose2d(60, 8, Math.toRadians(-180)), //starting pos
                new Pose2d(36, 8, Math.toRadians(90)), //firstPreIntake pos
                new Pose2d(36, 52, Math.toRadians(90)), //firstIntakePos
                new Pose2d(12, 8, Math.toRadians(-90)), //secondPreIntake pos
                new Pose2d(12, 52, Math.toRadians(90)), //secondIntake pos
                new Pose2d(60, 8, Math.toRadians(0)) //shooting pos
        ),
        FAR_RED(
                new Pose2d(-55, 43, Math.toRadians(-50)),
                new Pose2d(-12, 4, Math.toRadians(90)),
                new Pose2d(-12, 52, Math.toRadians(90)),
                new Pose2d(12, 8, Math.toRadians(-90)),
                new Pose2d(12, 52, Math.toRadians(90)),
                new Pose2d(-16, 8, Math.toRadians(0))
        ),
        FAR_BLUE(
                new Pose2d(-55, -43, Math.toRadians(50)),
                new Pose2d(-12, -4, Math.toRadians(-90)),
                new Pose2d(-12, -52, Math.toRadians(-90)),
                new Pose2d(12, -8, Math.toRadians(-90)),
                new Pose2d(12, -52, Math.toRadians(-90)),
                new Pose2d(-16, -8, Math.toRadians(0))
        );
        AutoPos(Pose2d startPos, Pose2d firstPreIntake, Pose2d firstIntake, Pose2d secondPreIntake, Pose2d secondIntake, Pose2d shootPos){
            this.startPos = startPos;
            this.firstPreIntake = firstPreIntake;
            this.firstIntake = firstIntake;
            this.secondPreIntake = secondPreIntake;
            this.secondIntake = secondIntake;
            this.shootPos = shootPos;
        }
        public final Pose2d startPos;
        public final Pose2d firstPreIntake;
        public final Pose2d firstIntake;
        public final Pose2d secondPreIntake;
        public final Pose2d secondIntake;
        public final Pose2d shootPos;

    }
    //Initial Position of Bot during auto
    //CB = closeBlue FB = farBlue CR = closeRed FR = farRed
    public void runOpMode() throws InterruptedException {
        Bot.instance = null;
        bot = Bot.getInstance(this);

        gp1 = new GamepadEx(gamepad1);
        MecanumDrive drive = new MecanumDrive(hardwareMap, chosenAuto.startPos);

        Action shootPreload = drive.actionBuilder(drive.localizer.getPose())
                .afterTime(0.1, bot.shootSetup())
                .build();

        Action intakeFirst = drive.actionBuilder(chosenAuto.startPos)
                .strafeToLinearHeading(new Vector2d(chosenAuto.firstPreIntake.component1().x, chosenAuto.firstPreIntake.component1().y), Math.toRadians(0), drive.defaultVelConstraint, drive.defaultAccelConstraint)
                .stopAndAdd(new InstantAction(() -> bot.intake.intake_without_sense()))
                .strafeToConstantHeading(new Vector2d(chosenAuto.firstIntake.component1().x, chosenAuto.firstIntake.component1().y), drive.defaultVelConstraint, drive.defaultAccelConstraint)
                .stopAndAdd(new InstantAction(() -> bot.intake.stopIntake()))
                .build();

        Action intakeSecond = drive.actionBuilder(chosenAuto.startPos)
                .strafeToLinearHeading(new Vector2d(chosenAuto.secondPreIntake.component1().x, chosenAuto.secondPreIntake.component1().y), Math.toRadians(0), drive.defaultVelConstraint, drive.defaultAccelConstraint)
                .stopAndAdd(new InstantAction(() -> bot.intake.intake_without_sense()))
                .strafeToConstantHeading(new Vector2d(chosenAuto.secondIntake.component1().x, chosenAuto.secondIntake.component1().y), drive.defaultVelConstraint, drive.defaultAccelConstraint)
                .stopAndAdd(new InstantAction(() -> bot.intake.stopIntake()))
                .build();

        Action shoot = drive.actionBuilder(new Pose2d(chosenAuto.shootPos.component1().x, chosenAuto.shootPos.component1().y, Math.toRadians(-150)))
                .splineToLinearHeading(chosenAuto.shootPos, Math.toRadians(-150), drive.defaultVelConstraint, drive.defaultAccelConstraint)
                .stopAndAdd(bot.shootSetup())
                .build();

        while (opModeInInit()) {
            gp1.readButtons();
            if (gp1.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER)) {
                bot.aprilTag.targetAllianceId = RED;
            }
            if (gp1.wasJustPressed(GamepadKeys.Button.LEFT_BUMPER)) {
                bot.aprilTag.targetAllianceId = BLUE;
            }
            if (gp1.wasJustPressed(GamepadKeys.Button.A)){
                chosenAuto = AutoPos.CLOSE_BLUE;
            }
            if (gp1.wasJustPressed(GamepadKeys.Button.B)){
                chosenAuto = AutoPos.CLOSE_RED;
            }
            if (gp1.wasJustPressed(GamepadKeys.Button.Y)){
                chosenAuto = AutoPos.FAR_RED;
            }
            if (gp1.wasJustPressed(GamepadKeys.Button.X)){
                chosenAuto = AutoPos.FAR_BLUE;
            }
        }
        waitForStart();
        while (opModeIsActive() && !isStopRequested()) {
                Actions.runBlocking(
                        new ActionHelper.RaceParallelCommand(
                                bot.actionPeriodic(),
                                new SequentialAction(
                                        shootPreload,
                                        intakeFirst,
                                        shoot,
                                        intakeSecond,
                                        shoot
                                )
                        )
                );

        }
    }
}
