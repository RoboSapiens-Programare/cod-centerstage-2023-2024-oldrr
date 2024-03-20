package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(400);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 50, Math.toRadians(180), Math.toRadians(180), 12)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-36, -60.5, Math.toRadians(-90)))
                                .setReversed(true)
                                .back(33)
                                .forward(11)
                                .turn(Math.toRadians(-90))
                                .forward(24)
                                .back(4)
                                .strafeRight(2)
                                .splineToConstantHeading(new Vector2d(47,-25), Math.toRadians(-20))
//                                .lineToLinearHeading(new Pose2d(-62,-7, Math.toRadians(-180)))
//                                .lineToLinearHeading(new Pose2d(24,-7, Math.toRadians(-180)))
//                                .splineToConstantHeading(new Vector2d(47,-25), Math.toRadians(0))
//                                .forward(2)
//                                .splineToConstantHeading(new Vector2d(45, -7), Math.toRadians(0))
//                                .lineToLinearHeading(new Pose2d(-62,-7, Math.toRadians(-290)))

                                .build()
                );
        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}