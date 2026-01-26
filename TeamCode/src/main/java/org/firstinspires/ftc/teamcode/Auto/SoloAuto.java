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

import java.util.ArrayList;

@Config
@Autonomous(name = "Solo Auto", group = "Autonomous")
public class SoloAuto extends LinearOpMode {
    Bot bot;
    private GamepadEx gp1;

    // boolean values for selective auto
        public boolean shootPreloadChosen = true;
        public boolean intakeAndShootFirstCloseChosen = true;
        public boolean intakeAndShootSecondCloseChosen = true;
        public boolean intakeAndShootThirdCloseChosen = true;
        public boolean intakeAndShootFirstFarChosen = true;
        public boolean intakeAndShootSecondFarChosen = true;
        public boolean intakeAndShootThirdFarChosen = true;
    private final int BLUE = 20, RED = 24, GPP = 21, PPG = 23, PGP = 22;
    private AutoPos chosenAuto;

    // Positions for different auto paths
    public enum AutoPos {
        CLOSE_BLUE(
                new Pose2d(-55, -43, Math.toRadians(-130)), //starting pos
                new Pose2d(-12, -22, Math.toRadians(-90)), //firstPreIntakePos
                new Pose2d(-12, -53, Math.toRadians(-90)), //firstIntakePos
                new Pose2d(12, -18, Math.toRadians(-90)), // secondPreIntakePos
                new Pose2d(12, -52, Math.toRadians(-90)), //secondIntakePos
                new Pose2d(36, -30, Math.toRadians(-90)), //thirdPreIntakePos
                new Pose2d(36, -52, Math.toRadians(-90)), //thirdIntakePos
                new Pose2d(-23, -16, Math.toRadians(-130)), //shootPos
                -90, //preIntakeTangentHeading
                140 // shootingTangentEnd

        ),
        CLOSE_RED(
                new Pose2d(-55, 43, Math.toRadians(130)), //starting pos
                new Pose2d(-12, 22, Math.toRadians(90)), //firstPreIntakePos
                new Pose2d(-12, 53, Math.toRadians(90)), //firstIntakePos
                new Pose2d(12, 18, Math.toRadians(90)), // secondPreIntakePos
                new Pose2d(12, 52, Math.toRadians(90)), //secondIntakePos
                new Pose2d(36, 30, Math.toRadians(90)), //thirdPreIntakePos
                new Pose2d(36, 52, Math.toRadians(90)), //thirdIntakePos
                new Pose2d(-23, 16, Math.toRadians(130)), //shootPos
                90, //preIntakeTangentHeading
                -140 // shootingTangentEnd
        ),
        FAR_BLUE(
                new Pose2d(60, -12, Math.toRadians(-180)), //starting pos
                new Pose2d(36, -36, Math.toRadians(-90)), //firstPreIntake pos change
                new Pose2d(36, -52, Math.toRadians(-90)), //firstIntakePos
                new Pose2d(12, -36, Math.toRadians(-90)), //secondPreIntake pos change
                new Pose2d(12, -52, Math.toRadians(-90)), //secondIntake pos
                new Pose2d(-12, -27, Math.toRadians(-90)), //thirdPreIntake pos
                new Pose2d(-12, -52, Math.toRadians(-90)), //thirdIntake pos
                new Pose2d(60, -14, Math.toRadians(-170)), //shooting pos
                -90, //preIntakeTangentHeading
                40 //shootingTangentEnd
        ),
        FAR_RED(
                new Pose2d(60, 12, Math.toRadians(-180)), //starting pos
                new Pose2d(36, 36, Math.toRadians(90)), //firstPreIntake pos change
                new Pose2d(36, 52, Math.toRadians(90)), //firstIntakePos
                new Pose2d(12, 36, Math.toRadians(90)), //secondPreIntake pos change
                new Pose2d(12, 52, Math.toRadians(90)), //secondIntake pos
                new Pose2d(-12, 27, Math.toRadians(90)), //thirdPreIntake pos
                new Pose2d(-12, 52, Math.toRadians(90)), //thirdIntake pos
                new Pose2d(60, 14, Math.toRadians(170)), //shooting pos
                90, //preIntakeTangentHeading
                -40 //shootingTangentEnd
        );
        //constructor
        AutoPos(Pose2d startPos, Pose2d firstPreIntake, Pose2d firstIntake, Pose2d secondPreIntake, Pose2d secondIntake, Pose2d thirdPreIntake, Pose2d thirdIntake, Pose2d shootPos, int preIntakeTangentHeading, int shootingTangentEnd) {
            this.startPos = startPos;
            this.firstPreIntake = firstPreIntake;
            this.firstIntake = firstIntake;
            this.secondPreIntake = secondPreIntake;
            this.secondIntake = secondIntake;
            this.thirdPreIntake = thirdPreIntake;
            this.thirdIntake = thirdIntake;
            this.shootPos = shootPos;
            this.preIntakeTangentHeading = preIntakeTangentHeading;
            this.shootingTangentEnd = shootingTangentEnd;

        }
        //final variables
        public final Pose2d startPos;
        public final Pose2d firstPreIntake;
        public final Pose2d firstIntake;
        public final Pose2d secondPreIntake;
        public final Pose2d secondIntake;
        public final Pose2d thirdPreIntake;
        public final Pose2d thirdIntake;
        public final Pose2d shootPos;
        public final int preIntakeTangentHeading;
        public final int shootingTangentEnd;

    }
    public void runOpMode() throws InterruptedException {
        Bot.instance = null;
        bot = Bot.getInstance(this);

        gp1 = new GamepadEx(gamepad1);

        // selects the chosenAuto
        while (opModeInInit() &&!isStarted()) {
            gp1.readButtons();
            if (gp1.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER)) {
                bot.aprilTag.targetAllianceId = RED;
            }
            if (gp1.wasJustPressed(GamepadKeys.Button.LEFT_BUMPER)) {
                bot.aprilTag.targetAllianceId = BLUE;
            }
            if (gp1.wasJustPressed(GamepadKeys.Button.A)) {
                chosenAuto = AutoPos.CLOSE_BLUE;
            }
            if (gp1.wasJustPressed(GamepadKeys.Button.B)) {
                chosenAuto = AutoPos.CLOSE_RED;
            }
            if (gp1.wasJustPressed(GamepadKeys.Button.Y)) {
                chosenAuto = AutoPos.FAR_RED;
            }
            if (gp1.wasJustPressed(GamepadKeys.Button.X)) {
                chosenAuto = AutoPos.FAR_BLUE;
            }
        }

        MecanumDrive drive = new MecanumDrive(hardwareMap, chosenAuto.startPos);

        // actions: shootPreload, shoot first three balls, shoot second three balls, shoot third three balls
        Action shootPreload = drive.actionBuilder(chosenAuto.startPos)
                .strafeToLinearHeading(new Vector2d(chosenAuto.shootPos.position.x, chosenAuto.shootPos.position.y),
                       chosenAuto.shootPos.heading)
                .afterTime(0.1, bot.actionOpenGate())
                .build();

        Action intakeAndShootFirstClose = drive.actionBuilder(chosenAuto.shootPos)
                .splineToLinearHeading(new Pose2d(chosenAuto.firstPreIntake.position, chosenAuto.firstPreIntake.heading), Math.toRadians(chosenAuto.preIntakeTangentHeading), drive.defaultVelConstraint, drive.defaultAccelConstraint)
                .stopAndAdd(new InstantAction(() -> bot.intake.intake_without_sense()))
                .strafeToLinearHeading(new Vector2d(chosenAuto.firstIntake.position.x, chosenAuto.firstIntake.position.y), chosenAuto.firstIntake.heading, drive.defaultVelConstraint, drive.defaultAccelConstraint)
                .stopAndAdd(new InstantAction(() -> bot.intake.stopIntake()))
                .setReversed(true)
                .splineToLinearHeading(chosenAuto.shootPos, Math.toRadians(chosenAuto.shootingTangentEnd), drive.defaultVelConstraint, drive.defaultAccelConstraint)
                .stopAndAdd(bot.actionOpenGate())
                .build();

        Action intakeAndShootSecondClose = drive.actionBuilder(chosenAuto.shootPos)
                .splineToLinearHeading(new Pose2d(chosenAuto.secondPreIntake.position, chosenAuto.secondPreIntake.heading), Math.toRadians(chosenAuto.preIntakeTangentHeading), drive.defaultVelConstraint, drive.defaultAccelConstraint)
                .stopAndAdd(new InstantAction(() -> bot.intake.intake_without_sense()))
                .strafeToLinearHeading(new Vector2d(chosenAuto.secondIntake.position.x, chosenAuto.secondIntake.position.y), chosenAuto.secondIntake.heading, drive.defaultVelConstraint, drive.defaultAccelConstraint)
                .stopAndAdd(new InstantAction(() -> bot.intake.stopIntake()))
                .setReversed(true)
                .splineToLinearHeading(chosenAuto.shootPos, Math.toRadians(chosenAuto.shootingTangentEnd), drive.defaultVelConstraint, drive.defaultAccelConstraint)
                .stopAndAdd(bot.actionOpenGate())
                .build();

        Action intakeAndShootThirdClose = drive.actionBuilder(chosenAuto.shootPos)
                .splineToLinearHeading(new Pose2d(chosenAuto.thirdPreIntake.position, chosenAuto.thirdPreIntake.heading), Math.toRadians(chosenAuto.preIntakeTangentHeading), drive.defaultVelConstraint, drive.defaultAccelConstraint)
                .stopAndAdd(new InstantAction(()-> bot.intake.intake_without_sense()))
                .strafeToLinearHeading(new Vector2d(chosenAuto.thirdIntake.position.x, chosenAuto.thirdIntake.position.y), chosenAuto.thirdIntake.heading, drive.defaultVelConstraint, drive.defaultAccelConstraint)
                .stopAndAdd(new InstantAction(() -> bot.intake.stopIntake()))
                .setReversed(true)
                .splineToLinearHeading(chosenAuto.shootPos, Math.toRadians(chosenAuto.shootingTangentEnd), drive.defaultVelConstraint, drive.defaultAccelConstraint)
                .stopAndAdd(bot.actionOpenGate())
                .build();

        Action intakeAndShootFirstFar = drive.actionBuilder(chosenAuto.shootPos)
                .splineTo(new Vector2d(chosenAuto.firstPreIntake.position.x,chosenAuto.firstPreIntake.position.y), chosenAuto.firstPreIntake.heading)
                .stopAndAdd(new InstantAction(() -> bot.intake.intake_without_sense()))
                .strafeToLinearHeading(new Vector2d(chosenAuto.firstIntake.position.x, chosenAuto.firstIntake.position.y), chosenAuto.firstIntake.heading, drive.defaultVelConstraint, drive.defaultAccelConstraint)
                .stopAndAdd(new InstantAction(() -> bot.intake.stopIntake()))
                .setReversed(true)
                .splineToLinearHeading(chosenAuto.shootPos, Math.toRadians(chosenAuto.shootingTangentEnd), drive.defaultVelConstraint, drive.defaultAccelConstraint)
                .stopAndAdd(bot.actionOpenGate())
                .build();

        Action intakeAndShootSecondFar = drive.actionBuilder(chosenAuto.shootPos)
                .splineTo(new Vector2d(chosenAuto.secondPreIntake.position.x,chosenAuto.secondPreIntake.position.y), chosenAuto.secondPreIntake.heading)
                .stopAndAdd(new InstantAction(() -> bot.intake.intake_without_sense()))
                .strafeToLinearHeading(new Vector2d(chosenAuto.secondIntake.position.x, chosenAuto.secondIntake.position.y), chosenAuto.secondIntake.heading, drive.defaultVelConstraint, drive.defaultAccelConstraint)
                .stopAndAdd(new InstantAction(() -> bot.intake.stopIntake()))
                .setReversed(true)
                .splineToLinearHeading(chosenAuto.shootPos, Math.toRadians(chosenAuto.shootingTangentEnd), drive.defaultVelConstraint, drive.defaultAccelConstraint)
                .stopAndAdd(bot.actionOpenGate())
                .build();

        Action intakeAndShootThirdFar = drive.actionBuilder(chosenAuto.shootPos)
                .splineToLinearHeading(new Pose2d(chosenAuto.thirdPreIntake.position, chosenAuto.thirdPreIntake.heading), Math.toRadians(chosenAuto.preIntakeTangentHeading), drive.defaultVelConstraint, drive.defaultAccelConstraint)
                .stopAndAdd(new InstantAction(()-> bot.intake.intake_without_sense()))
                .strafeToLinearHeading(new Vector2d(chosenAuto.thirdIntake.position.x, chosenAuto.thirdIntake.position.y), chosenAuto.thirdIntake.heading, drive.defaultVelConstraint, drive.defaultAccelConstraint)
                .stopAndAdd(new InstantAction(() -> bot.intake.stopIntake()))
                .setReversed(true)
                .splineToLinearHeading(chosenAuto.shootPos, Math.toRadians(chosenAuto.shootingTangentEnd), drive.defaultVelConstraint, drive.defaultAccelConstraint)
                .stopAndAdd(bot.actionOpenGate())
                .build();


        waitForStart();
            //identifies chosenAuto and performs it's respective list of actions
            if (chosenAuto == AutoPos.CLOSE_BLUE || chosenAuto == AutoPos.CLOSE_RED) {
                ArrayList<Action> actionListClose = new ArrayList<>();
                if (shootPreloadChosen) actionListClose.add(shootPreload);
                if (intakeAndShootFirstCloseChosen) actionListClose.add(intakeAndShootFirstClose);
                if (intakeAndShootSecondCloseChosen) actionListClose.add(intakeAndShootSecondClose);
                if (intakeAndShootThirdCloseChosen) actionListClose.add(intakeAndShootThirdClose);
                Actions.runBlocking(
                        new ActionHelper.RaceParallelCommand(
                                bot.actionPeriodic(),
                                new SequentialAction(
                                        actionListClose
                                )
                        )
                );
            } else {
                ArrayList<Action> actionListFar = new ArrayList<>();
                if (shootPreloadChosen) actionListFar.add(shootPreload);
                if (intakeAndShootFirstFarChosen) actionListFar.add(intakeAndShootFirstFar);
                if (intakeAndShootSecondFarChosen) actionListFar.add(intakeAndShootSecondFar);
                if (intakeAndShootThirdFarChosen) actionListFar.add(intakeAndShootThirdFar);
                Actions.runBlocking(
                        new ActionHelper.RaceParallelCommand(
                                bot.actionPeriodic(),
                                new SequentialAction(
                                        actionListFar
                                )
                        )
                );
            }
        }

}


