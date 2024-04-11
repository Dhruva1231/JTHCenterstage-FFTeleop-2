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

                                .splineTo(new Vector2d(4, 0), Math.toRadians(0))
                                .splineTo(new Vector2d(23, -10), Math.toRadians(-50))
//                .addDisplacementMarker(() -> {
//                    frontgrab.setPosition(0.4);
//                })
                                .setReversed(true)
                                .back(2)
//                                .splineTo(new Vector2d(26, 10), Math.toRadians(90))
//                                .splineTo(new Vector2d(23, 30), Math.toRadians(90))
//                                .splineTo(new Vector2d(28, 38), Math.toRadians(90))
                                .lineToLinearHeading(new Pose2d(28, 38, Math.toRadians(-90)))

                                .waitSeconds(1)
                                .setReversed(false)
                                .splineTo(new Vector2d(-1, 18), Math.toRadians(-93))
                                .lineTo(new Vector2d(-1, -40))
                                .splineTo(new Vector2d(13.85, -67.85), Math.toRadians(-80))
                                .waitSeconds(0.1)
                                .setReversed(true)
                                .splineTo(new Vector2d(-1, -40), Math.toRadians(93))
                                .splineTo(new Vector2d(-1, 0), Math.toRadians(93))
                                .splineTo(new Vector2d(16.5, 40), Math.toRadians(90))



                                //                                .splineToConstantHeading(new Vector2d(19, -68.5), Math.toRadians(0))
//                                .splineTo(new Vector2d(19, -68.5), Math.toRadians(-93))
//                                .splineTo(new Vector2d(-1, 18), Math.toRadians(-93))
//                                .splineTo(new Vector2d(-1, -40), Math.toRadians(-93))
////                                .splineTo(new Vector2d(18, -63.5), Math.toRadians(-93))
//                                .splineTo(new Vector2d(5, -55), Math.toRadians(-70))
//                                .splineTo(new Vector2d(17, -63.5), Math.toRadians(-70))
//                                .lineToSplineHeading(new Pose2d(15, -63.5, Math.toRadians(-70)))
//                                .lineToSplineHeading(new Pose2d(17, -68.5, Math.toRadians(-70)))

                                //                                .splineTo(new Vector2d(19, -68.5), Math.toRadians(-70))

//                                .splineTo(new Vector2d(19, -63.5), Math.toRadians(-93))
//                                .splineTo(new Vector2d(19, -68.5), Math.toRadians(-93))

                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}