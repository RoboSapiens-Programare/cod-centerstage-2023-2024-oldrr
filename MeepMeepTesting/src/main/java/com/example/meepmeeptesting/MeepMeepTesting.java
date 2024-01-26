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
                .setConstraints(40, 40, Math.toRadians(180), Math.toRadians(180), 17.5)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-36, -60, Math.toRadians(-90)))
                                .setReversed(true)
                                .back(10)
                                .strafeRight(2)
                                .lineToLinearHeading(new Pose2d(-30.5,-36, Math.toRadians(-130)))
                                .forward(8)
                                .splineToLinearHeading(new Pose2d(-54,-27.3, Math.toRadians(-180)), Math.toRadians(180))
                                .forward(5)
                                .back(2)
                                .strafeRight(18)
                                .back(68)
                                .splineToLinearHeading(new Pose2d(40,-39.5, Math.toRadians(-180)), Math.toRadians(-90))
                                .back(10)
                                .forward(4)
                                .strafeLeft(22)
                                .build()
                );
        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}