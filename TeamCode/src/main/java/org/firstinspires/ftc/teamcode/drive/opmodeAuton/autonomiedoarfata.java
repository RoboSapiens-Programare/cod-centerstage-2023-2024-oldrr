package org.firstinspires.ftc.teamcode.drive.opmodeAuton;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.robot.MecanumRobot;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.util.PoseStorage;

@Autonomous(name = "Autonomie roadrunner rosu doar fata departe", group="autonomous")

public class autonomiedoarfata extends LinearOpMode {
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

//        robot.intake.ridicaSweeper();
        robot.outtake.susCuva();
        robot.outtake.coboaraCuva();

        while (opModeIsActive()) {
            Pose2d start = new Pose2d(12, 60.5, Math.toRadians(90));
            robot.drive.setPoseEstimate(start);
            TrajectorySequence myTrajectory1 = robot.drive.trajectorySequenceBuilder(start)
                    .setReversed(true)
                    .back(33)
                    .waitSeconds(1000)
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
