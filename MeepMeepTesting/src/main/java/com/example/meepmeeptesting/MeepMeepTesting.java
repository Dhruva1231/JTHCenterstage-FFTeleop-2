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
                                .splineToLinearHeading(new Pose2d(37, -3.5, Math.toRadians(-72)), Math.toRadians(0))
                                .waitSeconds(0.1)
                                .splineToLinearHeading(new Pose2d(50, -4, Math.toRadians(-90)), Math.toRadians(0))
                                .waitSeconds(0.01)
                                .setReversed(true)
                                .splineTo(new Vector2d(50, 30), Math.toRadians(95))
                                .splineToConstantHeading(new Vector2d(24, 53), Math.toRadians(95))
                                .splineToConstantHeading(new Vector2d(20, 53), Math.toRadians(95))
                                .waitSeconds(0.1)
                                .lineToLinearHeading(new Pose2d(47, 40, Math.toRadians(-92)))
                                .lineToLinearHeading(new Pose2d(47, 15, Math.toRadians(-92)))

//                                .splineTo(new Vector2d(24, 53), Math.toRadians(95))

                                //                                .splineTo(new Vector2d(30, -2), Math.toRadians(0))
//                                .splineTo(new Vector2d(37, -4), Math.toRadians(-63))
//

                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}