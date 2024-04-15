package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(0, 0, Math.toRadians(0)))
//                                .lineToLinearHeading(new Pose2d(23, 13, Math.toRadians(-45)))
//                                .setReversed(true)
//                                .lineToLinearHeading(new Pose2d(22, 39, Math.toRadians(-90)))
//                                .waitSeconds(0.1)
//                                .setReversed(false)
//                                .splineTo(new Vector2d(-1, 18), Math.toRadians(-93))
//                                .lineTo(new Vector2d(-1, -40))
//                                .splineTo(new Vector2d(14.5, -67.85), Math.toRadians(-80))
//                                .waitSeconds(0.1)
//                                .setReversed(true)
//                                .splineTo(new Vector2d(-1, -40), Math.toRadians(87))
//                                .splineTo(new Vector2d(-1, 0), Math.toRadians(87))
//                                .splineTo(new Vector2d(18, 40), Math.toRadians(87))
//                                .waitSeconds(0.1)
//                                .setReversed(false)
//                                .splineTo(new Vector2d(-1.5, 18), Math.toRadians(-90))
//                                .lineTo(new Vector2d(-1.5, -40))
//                                .splineTo(new Vector2d(13.5, -68), Math.toRadians(-80))
//                                .waitSeconds(0.1)
//                                .setReversed(true)
//                                .splineTo(new Vector2d(-1.5, -40), Math.toRadians(87))
//                                .splineTo(new Vector2d(-1.5, 0), Math.toRadians(87))
//                                .splineTo(new Vector2d(16, 40), Math.toRadians(87))
//                                .waitSeconds(0.25)
//                                .setReversed(false)
//                                .splineTo(new Vector2d(-9, 30), Math.toRadians(-93))
//                                .lineTo(new Vector2d(-9, 45))

                                .lineToLinearHeading(new Pose2d(23, -13, Math.toRadians(45)))
                                .setReversed(true)
                                .lineToLinearHeading(new Pose2d(22, -39, Math.toRadians(90)))
                                .waitSeconds(0.1)
                                .setReversed(false)
                                .splineTo(new Vector2d(-1, -18), Math.toRadians(93))
                                .lineTo(new Vector2d(-1, 40))
                                .splineTo(new Vector2d(14.5, 67.85), Math.toRadians(80))
                                .waitSeconds(0.1)
                                .setReversed(true)
                                .splineTo(new Vector2d(-1, 40), Math.toRadians(-87))
                                .splineTo(new Vector2d(-1, 0), Math.toRadians(-87))
                                .splineTo(new Vector2d(18, -40), Math.toRadians(-87))
                                .waitSeconds(0.1)
                                .setReversed(false)
                                .splineTo(new Vector2d(-1.5, -18), Math.toRadians(90))
                                .lineTo(new Vector2d(-1.5, 40))
                                .splineTo(new Vector2d(13.5, 68), Math.toRadians(80))
                                .waitSeconds(0.1)
                                .setReversed(true)
                                .splineTo(new Vector2d(-1.5, 40), Math.toRadians(-87))
                                .splineTo(new Vector2d(-1.5, 0), Math.toRadians(-87))
                                .splineTo(new Vector2d(16, -40), Math.toRadians(-87))
                                .waitSeconds(0.25)
                                .setReversed(false)
                                .splineTo(new Vector2d(-9, -30), Math.toRadians(93))
                                .lineTo(new Vector2d(-9, -45))


                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}