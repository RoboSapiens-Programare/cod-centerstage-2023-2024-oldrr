/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.drive.opmodeAuton;



import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.robot.MecanumRobot;
import org.firstinspires.ftc.teamcode.drive.vision.OpenCVThreadRosuAproape;
import org.firstinspires.ftc.teamcode.drive.vision.OpenCVThreadRosuDeparte;
import org.firstinspires.ftc.teamcode.drive.vision.PiramidaAlbastruDeparte;
import org.firstinspires.ftc.teamcode.drive.vision.PiramidaRosuAproape;
import org.firstinspires.ftc.teamcode.drive.vision.PiramidaRosuDeparte;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.util.PoseStorage;


/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When a selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Autonomous(name = "Autonomie roadrunner sigur rosu departe", group="autonomous")

public class Autonomiesigurarosudeparte extends LinearOpMode {

//    Declare OpMode members.
//    private ElapsedTime runtime = new ElapsedTime();

    private MecanumRobot robot = null;
    public OpenCVThreadRosuDeparte openCV;
    public ElapsedTime opencvTimer;
    public static int MAX_MILISECONDS = 5000;
    private PiramidaRosuDeparte.Location finalLocation;


    public void runOpMode() {
        telemetry.addData(">", "Initializing...");
        telemetry.update();
        robot = new MecanumRobot(hardwareMap);

        openCV = new OpenCVThreadRosuDeparte(hardwareMap);
        finalLocation = PiramidaRosuDeparte.Location.LEFT;

        openCV.start();

        telemetry.addData("has initialised", "yes");
        telemetry.update();
        while (robot.isInitialize() && opModeIsActive()) {
            idle();
        }

        telemetry.addData(">", "Initialized");
        telemetry.update();

        opencvTimer = new ElapsedTime();
        opencvTimer.startTime();

        while (!isStarted()) {
            finalLocation = openCV.getLocation();
            telemetry.addData("Location: ", finalLocation);
            telemetry.update();
        }

        telemetry.setMsTransmissionInterval(50);

        /*
         * The INIT-loop:
         * This REPLACES waitForStart!
         */



        while (opModeIsActive()) {
            //modifica asta daca alianta e mai rapida ca noi ca suntem niste sclavi
//            sleep(2000);
//            robot.intake.ridicaSweeper();
            robot.outtake.susCuva();
            robot.outtake.coboaraCuva();
            Pose2d start = new Pose2d(-36, -60.5, Math.toRadians(-90));
            robot.drive.setPoseEstimate(start);
            if(finalLocation == PiramidaRosuDeparte.Location.RIGHT){
                TrajectorySequence myTrajectory1 = robot.drive.trajectorySequenceBuilder(start)
                        .setReversed(true)
                        .back(10)
                        .strafeRight(2)
                        .lineToLinearHeading(new Pose2d(-30.5,-36, Math.toRadians(-130)))
                        .forward(4)
                        .lineToLinearHeading(new Pose2d(-44,-10, Math.toRadians(-180)))
                        .back(68)
                        .splineToLinearHeading(new Pose2d(40,-39.5, Math.toRadians(-180)), Math.toRadians(-90))
                        .addDisplacementMarker(() -> {
                            robot.outtake.manualLevel(320);
                            robot.outtake.ridicaCuva();
                            robot.outtake.susCuva();
                        })
                        .waitSeconds(1)
                        .back(10)
                        .waitSeconds(0.2)
                        .addTemporalMarker(() ->{
                            robot.outtake.servoCuvaGecko.setPower(-0.75);
                        })
                        .waitSeconds(1)
                        .addDisplacementMarker(() -> {
                            robot.outtake.manualLevel(500);
                        })
                        .waitSeconds(0.2)
                        .forward(6)
                        .strafeLeft(18)
                        .addTemporalMarker(() -> {
                            robot.outtake.servoCuvaGecko.setPower(0);
                            robot.outtake.coboaraCuva();
                            sleep(200);
                            robot.outtake.manualLevel(-20);
                            sleep(200);
                        })
                        .build();
                robot.drive.followTrajectorySequence(myTrajectory1);
                sleep(30000);
            }
            else if(finalLocation == PiramidaRosuDeparte.Location.CENTER){
                TrajectorySequence myTrajectory1 = robot.drive.trajectorySequenceBuilder(start)
                        .setReversed(true)
                        .back(26)
                        .lineToLinearHeading(new Pose2d(-46,-46, Math.toRadians(-180)))
                        .lineToLinearHeading(new Pose2d(-54,-8, Math.toRadians(-180)))
                        .back(75)
                        .splineToConstantHeading(new Vector2d(40,-34), Math.toRadians(0))
                        .addDisplacementMarker(() -> {
                            robot.outtake.manualLevel(320);
                            robot.outtake.ridicaCuva();
                            robot.outtake.susCuva();
                        })
                        .waitSeconds(1)
                        .back(13)
                        .waitSeconds(0.15)
                        .addTemporalMarker(() ->{
                            robot.outtake.servoCuvaGecko.setPower(-0.75);
                        })
                        .waitSeconds(1)
                        .addDisplacementMarker(() -> {
                            robot.outtake.manualLevel(500);
                        })
                        .waitSeconds(0.2)
                        .forward(4)
                        .strafeLeft(24)
                        .addTemporalMarker(() -> {
                            robot.outtake.servoCuvaGecko.setPower(0);
                            robot.outtake.coboaraCuva();
                            sleep(200);
                            robot.outtake.manualLevel(-20);
                            sleep(200);
                        })
                        .build();
                robot.drive.followTrajectorySequence(myTrajectory1);
                sleep(30000);
            }
            else {
                TrajectorySequence myTrajectory1 = robot.drive.trajectorySequenceBuilder(start)
                        .setReversed(true)
                        .splineToLinearHeading(new Pose2d(-46,-35,Math.toRadians(-90)), Math.toRadians(90))
                        .forward(4)
                        .strafeLeft(12)
                        .lineToConstantHeading(new Vector2d(-35,-8))
                        .lineToLinearHeading(new Pose2d(24,-8, Math.toRadians(-180)))
                        .splineToConstantHeading(new Vector2d(40,-27), Math.toRadians(0))
                        .addDisplacementMarker(() -> {
                            robot.outtake.manualLevel(320);
                            robot.outtake.ridicaCuva();
                            robot.outtake.susCuva();
                        })
                        .waitSeconds(1)
                        .back(9)
                        .waitSeconds(0.2)
                        .addTemporalMarker(() ->{
                            robot.outtake.servoCuvaGecko.setPower(-0.75);
                        })
                        .waitSeconds(1)
                        .addDisplacementMarker(() -> {
                            robot.outtake.manualLevel(500);
                        })
                        .waitSeconds(0.2)
                        .forward(4)
                        .strafeLeft(28)
                        .addTemporalMarker(() -> {
                            robot.outtake.servoCuvaGecko.setPower(0);
                            robot.outtake.coboaraCuva();
                            sleep(200);
                            robot.outtake.manualLevel(-20);
                            sleep(200);
                        })
                        .build();
                robot.drive.followTrajectorySequence(myTrajectory1);
                sleep(30000);
            }
            if (isStopRequested()) {
                stop();
            }
        }

        PoseStorage.currentPose = robot.drive.getPoseEstimate();

//        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.update();
    }
}