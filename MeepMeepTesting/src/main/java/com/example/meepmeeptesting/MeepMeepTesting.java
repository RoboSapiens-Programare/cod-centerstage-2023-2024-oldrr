package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(500);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(40, 40, Math.toRadians(180), Math.toRadians(180), 17.5)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(12, -60, Math.toRadians(-90)))
                                .setReversed(true)
                                .splineToConstantHeading(new Vector2d(21, -40), Math.toRadians(90))
                                .back(4)
                                .forward(12)
                                .splineToLinearHeading(new Pose2d(43,-39, Math.toRadians(-180)), Math.toRadians(90))
                                .back(4.5)
                                .forward(4)
                                .strafeLeft(20)
                                .forward(70)
                                .splineToLinearHeading(new Pose2d(-56,-29, Math.toRadians(-180)), Math.toRadians(90))
                                .forward(4)
                                .back(4)
                                .strafeRight(18)
                                .back(102)
                                .lineToSplineHeading(new Pose2d(44,-35, Math.toRadians(-180)))
                                .back(5)
                                .build()
                );
        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}