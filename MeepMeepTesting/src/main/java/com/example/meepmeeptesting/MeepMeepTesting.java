package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(60, -8, Math.toRadians(-180)))
                .splineTo(new Vector2d(36,-36), Math.toRadians(-90))
                .strafeToLinearHeading(new Vector2d(36,-52), Math.toRadians(-90))
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(60,-8,Math.toRadians(-150)),Math.toRadians(40))
                        .waitSeconds(3)
                .splineTo(new Vector2d(12,-36), Math.toRadians(-90))
                .strafeToLinearHeading(new Vector2d(12,-52), Math.toRadians(-90))
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(60,-8,Math.toRadians(-150)),Math.toRadians(40))
                        .waitSeconds(3)
                .build());

        RoadRunnerBotEntity myBot2 = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();
        myBot2.runAction(myBot.getDrive().actionBuilder(new Pose2d(60, 8, Math.toRadians(-180)))
                .strafeToLinearHeading(new Vector2d(36,8), Math.toRadians(90))
                .strafeToLinearHeading(new Vector2d(36,52), Math.toRadians(90))
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(60,8,Math.toRadians(150)),Math.toRadians(140))
                .waitSeconds(3)
                .strafeToLinearHeading(new Vector2d(12,8), Math.toRadians(90))
                .strafeToLinearHeading(new Vector2d(12,52), Math.toRadians(90))
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(60,8,Math.toRadians(150)),Math.toRadians(140))
                .waitSeconds(3)
                .build());

        RoadRunnerBotEntity myBot3 = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();
        myBot3.runAction(myBot.getDrive().actionBuilder(new Pose2d(-55, 43, Math.toRadians(-50)))
                        .setTangent(Math.toRadians(-20))
                .splineTo(new Vector2d(-12,15), Math.toRadians(90))
                .strafeToLinearHeading(new Vector2d(-12,55), Math.toRadians(90))
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(-27,14,Math.toRadians(130)),Math.toRadians(140))
                .waitSeconds(3)
                        .setReversed(true)
                .splineToLinearHeading(new Pose2d(12,15,Math.toRadians(90)), Math.toRadians(90))
                .strafeToLinearHeading(new Vector2d(12,52), Math.toRadians(90))
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(-16,15,Math.toRadians(130)),Math.toRadians(140))
                .waitSeconds(3)
                .build());

        RoadRunnerBotEntity myBot4 = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();
        myBot4.runAction(myBot.getDrive().actionBuilder(new Pose2d(-55, -43, Math.toRadians(-50)))
                .strafeToLinearHeading(new Vector2d(-12,-4), Math.toRadians(-90))
                .strafeToLinearHeading(new Vector2d(-12,-53), Math.toRadians(-90))
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(-27,-14,Math.toRadians(-130)),Math.toRadians(140))
                .waitSeconds(3)
                .strafeToLinearHeading(new Vector2d(12,-8), Math.toRadians(-90))
                .strafeToLinearHeading(new Vector2d(12,-52), Math.toRadians(-90))
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(-16,-8,Math.toRadians(-130)),Math.toRadians(140))
                .waitSeconds(3)
                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_DECODE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot3)
//                .addEntity(myBot2)
//                .addEntity(myBot3)
//                .addEntity(myBot4)
                .start();
    }
}