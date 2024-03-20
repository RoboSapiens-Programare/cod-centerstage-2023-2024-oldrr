package org.firstinspires.ftc.teamcode.drive.opmodeAuton;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.robot.MecanumRobot;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.util.PoseStorage;

@Autonomous(name = "Autonomie roadrunner albastru aproape", group="autonomous")

public class AutonomieRosuAproape extends LinearOpMode {
private MecanumRobot robot = null;

    public static int MAX_MILISECONDS = 5000;

    public void runOpMode() {
        telemetry.addData(">", "Initializing...");
        robot = new MecanumRobot(hardwareMap);

        telemetry.addData("has initialised", "yes");
        while (robot.isInitialize() && opModeIsActive()) {
            idle();
        }

        telemetry.addData(">", "Initialized");
        telemetry.setMsTransmissionInterval(50);

        telemetry.update();
        waitForStart();

        robot.intake.ridicaSweeper();
        robot.outtake.susCuva();
        robot.outtake.coboaraCuva();

        while (opModeIsActive()) {
            Pose2d start = new Pose2d(12, -60, Math.toRadians(-90));
            robot.drive.setPoseEstimate(start);
            TrajectorySequence myTrajectory1 = robot.drive.trajectorySequenceBuilder(start)
                    .setReversed(true)
                    .back(33)
                    .forward(4)
                    .strafeRight(26)
                    .back(24)
                    .addTemporalMarker(() -> {
                        robot.intake.setSweeper(0.75);
                    })
                    .lineToLinearHeading(new Pose2d(-62,-7, Math.toRadians(-180)))
                    .addTemporalMarker(() -> {
                            robot.intake.setSweeper(0.75);
                            robot.intake.setSweepPower(1);
                        })
                    .waitSeconds(3)
                    .addTemporalMarker(() -> {
                        robot.intake.ridicaSweeper();
                        robot.intake.setSweepPower(0);
                    })
                    .lineToLinearHeading(new Pose2d(24,-7, Math.toRadians(-180)))
                    .addTemporalMarker(() -> {
                        robot.outtake.manualLevel(600);
                    })
                    .addTemporalMarker(() -> {
                        robot.outtake.ridicaCuva();
                        robot.outtake.susCuva();
                    })
                    .waitSeconds(0.1)
                    .addTemporalMarker(() -> {
                        robot.outtake.dreaptaCuva();
                    })
                    .splineToConstantHeading(new Vector2d(47,-25), Math.toRadians(0))
                    .back(4.6)
                    .forward(6)
                    .splineToConstantHeading(new Vector2d(45, -7), Math.toRadians(0))
                    .addTemporalMarker(() -> {
                        robot.outtake.susCuva();
                        robot.outtake.coboaraCuva();
                    })
                    .strafeRight(15)
                    .addTemporalMarker(() -> {
                        robot.outtake.manualLevel(0);
                    })
                    .addTemporalMarker(() -> {
                        robot.intake.setSweeper(0.75);
                    })
                    .lineToLinearHeading(new Pose2d(-62,-7, Math.toRadians(-180)))
                    .addTemporalMarker(() -> {
                        robot.intake.setSweeper(0.75);
                        robot.intake.setSweepPower(1);
                    })
                    .waitSeconds(3)
                    .addTemporalMarker(() -> {
                        robot.intake.ridicaSweeper();
                        robot.intake.setSweepPower(0);
                    })
                    .lineToLinearHeading(new Pose2d(24,-7, Math.toRadians(-180)))
                    .addTemporalMarker(() -> {
                        robot.outtake.manualLevel(600);
                    })
                    .addTemporalMarker(() -> {
                        robot.outtake.ridicaCuva();
                        robot.outtake.susCuva();
                    })
                    .waitSeconds(0.1)
                    .addTemporalMarker(() -> {
                        robot.outtake.dreaptaCuva();
                    })
                    .splineToConstantHeading(new Vector2d(47,-25), Math.toRadians(0))
                    .back(4.6)
                    .waitSeconds(100)
                    .build();
        robot.drive.followTrajectorySequence(myTrajectory1);
            if (isStopRequested()) {
                stop();
            }
        }

        PoseStorage.currentPose = robot.drive.getPoseEstimate();

//        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.update();
    }
}
