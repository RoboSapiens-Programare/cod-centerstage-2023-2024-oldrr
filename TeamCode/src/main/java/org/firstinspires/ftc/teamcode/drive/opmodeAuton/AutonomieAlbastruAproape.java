package org.firstinspires.ftc.teamcode.drive.opmodeAuton;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.robot.MecanumRobot;
import org.firstinspires.ftc.teamcode.drive.vision.OpenCVThreadAlbastruAproape;
import org.firstinspires.ftc.teamcode.drive.vision.PiramidaAlbastruAproape;
import org.firstinspires.ftc.teamcode.drive.vision.PiramidaRosuAproape;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.util.PoseStorage;


@Autonomous(name = "Autonomie_ALBASTRU_APROAPE", group="autonomous")

public class AutonomieAlbastruAproape extends LinearOpMode {
private MecanumRobot robot = null;

    public static int MAX_MILISECONDS = 5000;
    public OpenCVThreadAlbastruAproape openCV;

    private PiramidaAlbastruAproape.Location finalLocation;

    public ElapsedTime opencvTimer;



    public void runOpMode() {
        telemetry.addData(">", "Initializing...");
        robot = new MecanumRobot(hardwareMap);

        openCV = new OpenCVThreadAlbastruAproape(hardwareMap);
        finalLocation = PiramidaAlbastruAproape.Location.LEFT;

        openCV.start();

        telemetry.addData("has initialised", "yes");
        while (robot.isInitialize() && opModeIsActive()) {
            idle();
        }

        telemetry.addData(">", "Initialized");
        telemetry.setMsTransmissionInterval(50);

        telemetry.update();

        opencvTimer = new ElapsedTime();
        opencvTimer.startTime();

        while (!isStarted()) {
            finalLocation = openCV.getLocation();
            telemetry.addData("Location: ", finalLocation);
            telemetry.update();
        }
        waitForStart();

        robot.intake.ridicaSweeper();
        robot.outtake.susCuva();
        robot.outtake.coboaraCuva();

        while (opModeIsActive()) {
            Pose2d start = new Pose2d(12, 63, Math.toRadians(90));
            robot.drive.setPoseEstimate(start);
            TrajectorySequence toRightThenBackdrop = robot.drive.trajectorySequenceBuilder(start)
                    .setReversed(true)
                    //get sweeper to correct line
//                    .lineToLinearHeading(new Pose2d(12, 27, Math.toRadians(90)))
                    .back(36)
                    .turn(Math.toRadians(90))
                    //take out pixel
                    .addTemporalMarker(() ->{
                        robot.intake.coboaraSweeper();
                        robot.intake.setSweepPower(-0.6);
                        robot.outtake.manualLevel(300);
                    })
                    .waitSeconds(0.3)
                    //stop sweeper and get outtake up
                    .addTemporalMarker(()->{
                        robot.intake.ridicaSweeper();
                        robot.outtake.ridicaCuva();
                        robot.intake.setSweepPower(0);
                    })
                    //go to backboard
                    .lineToLinearHeading(new Pose2d(54, 27, Math.toRadians(180)))
                    //drop pixel
                    .addTemporalMarker(()->{
                        robot.outtake.servoCuvaGecko.setPower(-0.2);
                    })
                    .waitSeconds(0.4)
                    .addTemporalMarker(()->{
                        robot.outtake.servoCuvaGecko.setPower(0);
                        robot.outtake.coboaraCuva();
                    })
                    .waitSeconds(0.4)
                    .addTemporalMarker(()->{
                        robot.outtake.manualLevel(-20);
                    })
                    .strafeRight(36)
                    .waitSeconds(100)
                    .build();

            TrajectorySequence toCenterThenBackdrop = robot.drive.trajectorySequenceBuilder(start)
                    .setReversed(true)
                    //get sweeper to correct line
                    .lineToLinearHeading(new Pose2d(25, 25, Math.toRadians(190)))
                    //take out pixel
                    .addTemporalMarker(() ->{
                        robot.intake.coboaraSweeper();
                        robot.intake.setSweepPower(-0.4);
                        robot.outtake.manualLevel(300);
                    })
                    .waitSeconds(0.3)
                    //stop sweeper and get outtake up
                    .addTemporalMarker(()->{
                        robot.intake.ridicaSweeper();
                        robot.outtake.ridicaCuva();
                        robot.intake.setSweepPower(0);
                    })
                    //go to backboard
                    .lineToLinearHeading(new Pose2d(54, 34, Math.toRadians(180)))
                    //drop pixel
                    .addTemporalMarker(()->{
                        robot.outtake.servoCuvaGecko.setPower(-0.2);
                    })
                    .waitSeconds(0.4)
                    .addTemporalMarker(()->{
                        robot.outtake.servoCuvaGecko.setPower(0);
                        robot.outtake.coboaraCuva();
                    })
                    .waitSeconds(0.4)
                    .addTemporalMarker(()->{
                        robot.outtake.manualLevel(-20);
                    })
                    .strafeRight(26)
                    .waitSeconds(100)
                    .build();

            TrajectorySequence toLeftThenBackdrop = robot.drive.trajectorySequenceBuilder(start)
                    .setReversed(true)
                    //get sweeper to correct line
                    .lineToLinearHeading(new Pose2d(35, 29, Math.toRadians(180)))
                    //take out pixel
                    .addTemporalMarker(() ->{
                        robot.intake.coboaraSweeper();
                        robot.intake.setSweepPower(-0.4);
                        robot.outtake.manualLevel(600);
                    })
                    .waitSeconds(0.25)
                    //stop sweeper and get outtake up
                    .addTemporalMarker(()->{
                        robot.intake.ridicaSweeper();
                        robot.outtake.ridicaCuva();
                        robot.intake.setSweepPower(0);
                    })
                    //go to backboard
                    .lineToLinearHeading(new Pose2d(53, 33, Math.toRadians(180)))
                    //drop pixel
                    .addTemporalMarker(()->{
                        robot.outtake.setCuva(0.2);
                    })
                    .waitSeconds(0.3)
                    .addTemporalMarker(()->{
                        robot.outtake.servoCuvaGecko.setPower(-0.2);
                    })
                    .waitSeconds(0.7)
                    .addTemporalMarker(()->{
                        robot.outtake.servoCuvaGecko.setPower(0);
                        robot.outtake.susCuva();

                    })
                    .waitSeconds(0.8)
                    .addTemporalMarker(()->{
                        robot.outtake.coboaraCuva();
                    })
                    .waitSeconds(0.4)
                    .addTemporalMarker(()->{
                        robot.outtake.manualLevel(-20);
                    })
                    .strafeRight(32)
                    .waitSeconds(100)
                    .build();
            if(finalLocation == PiramidaAlbastruAproape.Location.RIGHT) {
                robot.drive.followTrajectorySequence(toRightThenBackdrop);
            }
            else if (finalLocation == PiramidaAlbastruAproape.Location.CENTER) {
                robot.drive.followTrajectorySequence(toCenterThenBackdrop);
            } else if (finalLocation == PiramidaAlbastruAproape.Location.LEFT) {
                robot.drive.followTrajectorySequence(toLeftThenBackdrop);
            }
            /*robot.drive.setPoseEstimate(toLeftThenBackdrop.end());
            TrajectorySequence backdropToMiddleStackAndBack = robot.drive.trajectorySequenceBuilder(toLeftThenBackdrop.end())
                    .setReversed(false)
                    .turn(Math.toRadians(-35))
                    .splineTo(new Vector2d(-55, -7), Math.toRadians(180))
                    .setReversed(true)
                    .splineToConstantHeading(new Vector2d(47, -34), Math.toRadians(-40))
                    .waitSeconds(10)
                    .build();
                if (isStopRequested()) {
                    stop();
                }*/
            }

            PoseStorage.currentPose = robot.drive.getPoseEstimate();

    //        telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.update();
        }
}
