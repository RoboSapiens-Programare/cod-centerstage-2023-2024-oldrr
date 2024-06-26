///* Copyright (c) 2017 FIRST. All rights reserved.
// *
// * Redistribution and use in source and binary forms, with or without modification,
// * are permitted (subject to the limitations in the disclaimer below) provided that
// * the following conditions are met:
// *
// * Redistributions of source code must retain the above copyright notice, this list
// * of conditions and the following disclaimer.
// *
// * Redistributions in binary form must reproduce the above copyright notice, this
// * list of conditions and the following disclaimer in the documentation and/or
// * other materials provided with the distribution.
// *
// * Neither the name of FIRST nor the names of its contributors may be used to endorse or
// * promote products derived from this software without specific prior written permission.
// *
// * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
// * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
// * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
// * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
// * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
// * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
// */
//
//package org.firstinspires.ftc.teamcode.drive.opmodeAuton;
//
//
//
//import com.acmerobotics.roadrunner.geometry.Pose2d;
//import com.acmerobotics.roadrunner.geometry.Vector2d;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.Disabled;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.util.ElapsedTime;
//
//import org.firstinspires.ftc.teamcode.drive.robot.MecanumRobot;
//import org.firstinspires.ftc.teamcode.drive.vision.OpenCVThreadRosuAproape;
//import org.firstinspires.ftc.teamcode.drive.vision.PiramidaRosuAproape;
//import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
//import org.firstinspires.ftc.teamcode.util.PoseStorage;
//
//
///**
// * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
// * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
// * of the FTC Driver Station. When a selection is made from the menu, the corresponding OpMode
// * class is instantiated on the Robot Controller and executed.
// *
// * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
// * It includes all the skeletal structure that all linear OpModes contain.
// *
// * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
// * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
// */
//
//@Autonomous(name = "Autonomie roadrunner rosu aproape stack", group="autonomous")
//
//public class AutonomieRosuAproapeStack extends LinearOpMode {
//
////    Declare OpMode members.
////    private ElapsedTime runtime = new ElapsedTime();
//
//    private MecanumRobot robot = null;
//    public OpenCVThreadRosuAproape openCV;
//    public ElapsedTime opencvTimer;
//    public static int MAX_MILISECONDS = 5000;
//    private PiramidaRosuAproape.Location finalLocation;
//
//
//    public void runOpMode() {
//        telemetry.addData(">", "Initializing...");
//        telemetry.update();
//        robot = new MecanumRobot(hardwareMap);
//
//        openCV = new OpenCVThreadRosuAproape(hardwareMap);
//        finalLocation = PiramidaRosuAproape.Location.LEFT;
//
//        openCV.start();
//
//        telemetry.addData("has initialised", "yes");
//        telemetry.update();
//        while (robot.isInitialize() && opModeIsActive()) {
//            idle();
//        }
//
//        telemetry.addData(">", "Initialized");
//        telemetry.update();
//
//        opencvTimer = new ElapsedTime();
//        opencvTimer.startTime();
//
//        while (!isStarted()) {
//            finalLocation = openCV.getLocation();
//            telemetry.addData("Location: ", finalLocation);
//            telemetry.update();
//        }
//
//        telemetry.setMsTransmissionInterval(50);
//
//        /*
//         * The INIT-loop:
//         * This REPLACES waitForStart!
//         */
//
//
//
//        while (opModeIsActive()) {
//            //modifica asta daca alianta e mai rapida ca noi ca suntem niste sclavi
//            sleep(0);
//            robot.outtake.inchideCuva();
////            robot.intake.inchideGheara();
//            Pose2d start = new Pose2d(12, -60.5, Math.toRadians(-90));
//            robot.drive.setPoseEstimate(start);
//            if(finalLocation == PiramidaRosuAproape.Location.RIGHT){
//                TrajectorySequence myTrajectory1 = robot.drive.trajectorySequenceBuilder(start)
//                        .setReversed(true)
//                        .splineToConstantHeading(new Vector2d(21, -40), Math.toRadians(90))
//                        .back(4)
//                        .forward(12)
//                        .splineToLinearHeading(new Pose2d(43,-39, Math.toRadians(-180)), Math.toRadians(90))
//                        .waitSeconds(0.2)
//                        .addTemporalMarker(() -> {
//                            robot.outtake.manualLevel(680);
//                            robot.outtake.ridicaCuva();
//                        })
//                        .waitSeconds(0.1)
//                        .back(4.5)
//                        .addTemporalMarker(() ->{
//                            robot.outtake.deschideCuva();
//                        })
//                        .waitSeconds(0.1)
//                        .addDisplacementMarker(() -> {
//                            robot.outtake.manualLevel(900);
//                        })
//                        .waitSeconds(0.1)
//                        .forward(4)
//                        .waitSeconds(0.1)
//                        .addTemporalMarker(() -> {
//                            robot.outtake.inchideCuva();
//                        })
//                        .addDisplacementMarker(() -> {
//                            robot.outtake.inchideCuva();
//                        })
//                        .strafeLeft(18)
//                        .addTemporalMarker(() -> {
//                            robot.outtake.inchideCuva();
//                        })
//                        .addDisplacementMarker(() -> {
//                            robot.outtake.inchideCuva();
//                        })
//                        .addTemporalMarker(() -> {
//                            robot.outtake.coboaraCuva();
//                            sleep(200);
//                            robot.outtake.manualLevel(-50);
//                            sleep(500);
//                            robot.outtake.deschideCuva();
//                        })
//                        .forward(70)
//                        .addDisplacementMarker(() -> {
////                          robot.intake.deschideGheara();
//                          robot.outtake.deschideCuva();
//                        })
//                        .splineToLinearHeading(new Pose2d(-54,-27.5, Math.toRadians(-180)), Math.toRadians(90))
//                        .forward(5)
//                        .addTemporalMarker(() -> {
////                            robot.intake.inchideGhearapos(0.6);
//                            robot.intake.setSweepPower(0.6);
//                                robot.intake.activateConveyor(-1);
//
//                                sleep(500);
//                                robot.intake.setSweepPower(-0.7);
//                                sleep(250);
//                                robot.intake.setSweepPower(0.6);
//                                sleep(1500);
//                                robot.intake.setSweepPower(-0.7);
//                            robot.outtake.inchideCuva();
//                        })
//                        .back(2)
//                        .addDisplacementMarker(() -> {
//                            robot.intake.stopConveyor();
//                            robot.intake.setSweepPower(0);
//                        })
//                        .back(2)
//                        .addTemporalMarker(() -> {
////                            robot.intake.inchideGheara();
//                        })
//                        .strafeRight(18)
//                        .back(102)
//                        .lineToSplineHeading(new Pose2d(44,-35, Math.toRadians(-180)))
//                        .addTemporalMarker(() -> {
//                            robot.outtake.manualLevel(680);
//                            robot.outtake.ridicaCuva();
//                        })
//                        .back(6)
//                        .addTemporalMarker(() ->{
//                            robot.outtake.deschideCuva();
//                        })
//                        .waitSeconds(0.1)
//                        .addDisplacementMarker(() -> {
//                            robot.outtake.manualLevel(900);
//                        })
//                        .waitSeconds(0.1)
//                        .forward(4)
//                        .addTemporalMarker(() -> {
//                            robot.outtake.inchideCuva();
//                        })
//                        .addDisplacementMarker(() -> {
//                            robot.outtake.inchideCuva();
//                        })
//                        .addTemporalMarker(() -> {
//                            robot.outtake.coboaraCuva();
//                            sleep(200);
//                            robot.outtake.manualLevel(-50);
//                            sleep(500);
//                            robot.outtake.deschideCuva();
//                        })
//                        .build();
//                robot.drive.followTrajectorySequence(myTrajectory1);
//                robot.outtake.deschideCuva();
//                sleep(30000);
//            }
//            else if(finalLocation == PiramidaRosuAproape.Location.CENTER){
//                TrajectorySequence myTrajectory1 = robot.drive.trajectorySequenceBuilder(start)
//                        .back(28)
//                        .forward(4)
//                        .lineToSplineHeading(new Pose2d(30,-34, Math.toRadians(-180)))
//                        .addDisplacementMarker(() -> {
//                            robot.outtake.manualLevel(680);
//                            robot.outtake.ridicaCuva();
//                        })
//                        .back(18.5)
//                        .waitSeconds(0.15)
//                        .addTemporalMarker(() -> {
//                            robot.outtake.deschideCuva();
//                        })
//                        .waitSeconds(0.1)
//                        .addDisplacementMarker(() -> {
//                            robot.outtake.manualLevel(900);
//                        })
//                        .waitSeconds(0.1)
//                        .forward(4)
//                        .strafeLeft(23)
//                        .waitSeconds(0.1)
//                        .addTemporalMarker(() -> {
//                            robot.outtake.inchideCuva();
//                        })
//                        .addDisplacementMarker(() -> {
//                            robot.outtake.inchideCuva();
//                        })
//                        .addTemporalMarker(() -> {
//                            robot.outtake.inchideCuva();
//                        })
//                        .addDisplacementMarker(() -> {
//                            robot.outtake.inchideCuva();
//                        })
//                        .addTemporalMarker(() -> {
//                            robot.outtake.coboaraCuva();
//                            sleep(200);
//                            robot.outtake.manualLevel(-50);
//                            sleep(500);
//                            robot.outtake.deschideCuva();
//                        })
//                        .forward(70)
//                        .addDisplacementMarker(() -> {
////                            robot.intake.deschideGheara();
//                            robot.outtake.deschideCuva();
//                        })
//                        .splineToLinearHeading(new Pose2d(-54,-27.3, Math.toRadians(-180)), Math.toRadians(90))
//                        .forward(5)
//                        .addTemporalMarker(() -> {
////                            robot.intake.inchideGhearapos(0.6);
//                            robot.intake.setSweepPower(0.6);
//                            robot.intake.activateConveyor(-1);
//                            sleep(500);
//                            robot.intake.setSweepPower(-0.7);
//                            sleep(250);
//                            robot.intake.setSweepPower(0.6);
//                            sleep(1500);
//                            robot.intake.setSweepPower(-0.7);
//                            robot.outtake.inchideCuva();
//                        })
//                        .back(2)
//                        .addDisplacementMarker(() -> {
//                            robot.intake.stopConveyor();
//                            robot.intake.setSweepPower(0);
//                        })
//                        .back(2)
//                        .addTemporalMarker(() -> {
////                            robot.intake.inchideGheara();
//                        })
//                        .strafeRight(18)
//                        .back(102)
//                        .lineToSplineHeading(new Pose2d(44,-29, Math.toRadians(-180)))
//                        .addTemporalMarker(() -> {
//                            robot.outtake.manualLevel(750);
//                            robot.outtake.ridicaCuva();
//                        })
//                        .back(8)
//                        .addTemporalMarker(() ->{
//                            robot.outtake.deschideCuva();
//                        })
//                        .waitSeconds(0.1)
//                        .addDisplacementMarker(() -> {
//                            robot.outtake.manualLevel(900);
//                        })
//                        .waitSeconds(0.1)
//                        .forward(4)
//                        .addTemporalMarker(() -> {
//                            robot.outtake.inchideCuva();
//                        })
//                        .addDisplacementMarker(() -> {
//                            robot.outtake.inchideCuva();
//                        })
//                        .addTemporalMarker(() -> {
//                            robot.outtake.coboaraCuva();
//                            sleep(200);
//                            robot.outtake.manualLevel(-50);
//                            sleep(500);
//                            robot.outtake.deschideCuva();
//                        })
//                        .build();
//                robot.drive.followTrajectorySequence(myTrajectory1);
//                robot.outtake.deschideCuva();
//                sleep(30000);
//            }
//            else {
//                TrajectorySequence myTrajectory1 = robot.drive.trajectorySequenceBuilder(start)
//                        .setReversed(true)
//                        //.back(12)
//                        .splineToLinearHeading(new Pose2d(9, -36, Math.toRadians(-30)), Math.toRadians(-60))
//                        .lineToSplineHeading(new Pose2d(30,-27, Math.toRadians(-180)))
//                        .addDisplacementMarker(() -> {
//                            robot.outtake.manualLevel(680);
//                            robot.outtake.ridicaCuva();
//                        })
//                        .back(19.5)
//                        .waitSeconds(0.2)
//                        .addTemporalMarker(() ->{
//                            robot.outtake.deschideCuva();
//                        })
//                        .waitSeconds(0.1)
//                        .addDisplacementMarker(() -> {
//                            robot.outtake.manualLevel(900);
//                        })
//                        .waitSeconds(0.1)
//                        .forward(5)
//                        .strafeRight(17)
//                        .addTemporalMarker(() -> {
//                            robot.outtake.inchideCuva();
//                        })
//                        .addDisplacementMarker(() -> {
//                            robot.outtake.inchideCuva();
//                        })
//                        .addTemporalMarker(() -> {
//                            robot.outtake.inchideCuva();
//                        })
//                        .addDisplacementMarker(() -> {
//                            robot.outtake.inchideCuva();
//                        })
//                        .addTemporalMarker(() -> {
//                            robot.outtake.coboaraCuva();
//                            sleep(200);
//                            robot.outtake.manualLevel(-50);
//                            sleep(500);
//                            robot.outtake.deschideCuva();
//                        })
//                        .forward(70)
//                        .addDisplacementMarker(() -> {
////                            robot.intake.deschideGheara();
//                            robot.outtake.deschideCuva();
//                        })
//                        .splineToLinearHeading(new Pose2d(-54,-15, Math.toRadians(-180)), Math.toRadians(180))
//                        .forward(5)
//                        .addTemporalMarker(() -> {
////                            robot.intake.inchideGhearapos(0.6);
//                            robot.intake.setSweepPower(0.6);
//                            robot.intake.activateConveyor(-1);
//                            sleep(500);
//                            robot.intake.setSweepPower(-0.7);
//                            sleep(250);
//                            robot.intake.setSweepPower(0.6);
//                            sleep(1000);
//                            robot.intake.setSweepPower(-0.7);
//                            robot.outtake.inchideCuva();
//                        })
//                        .back(2)
//                        .addDisplacementMarker(() -> {
//                            robot.intake.stopConveyor();
//                            robot.intake.setSweepPower(0);
//                        })
//                        .back(2)
//                        .addTemporalMarker(() -> {
////                            robot.intake.inchideGheara();
//                        })
//                        .splineToLinearHeading(new Pose2d(-24,-10, Math.toRadians(-180)), Math.toRadians(0))
//                        .back(48)
//                        .splineToLinearHeading(new Pose2d(44,-35, Math.toRadians(-180)), Math.toRadians(0))
//                        .addTemporalMarker(() -> {
//                            robot.outtake.manualLevel(680);
//                            robot.outtake.ridicaCuva();
//                        })
//                        .back(8)
//                        .addTemporalMarker(() ->{
//                            robot.outtake.deschideCuva();
//                        })
//                        .waitSeconds(0.1)
//                        .addDisplacementMarker(() -> {
//                            robot.outtake.manualLevel(900);
//                        })
//                        .waitSeconds(0.1)
//                        .forward(4)
//                        .addTemporalMarker(() -> {
//                            robot.outtake.inchideCuva();
//                        })
//                        .addDisplacementMarker(() -> {
//                            robot.outtake.inchideCuva();
//                        })
//                        .addTemporalMarker(() -> {
//                            robot.outtake.coboaraCuva();
//                            sleep(200);
//                            robot.outtake.manualLevel(-50);
//                            sleep(500);
//                            robot.outtake.deschideCuva();
//                        })
//                        .build();
//                robot.drive.followTrajectorySequence(myTrajectory1);
//                robot.outtake.deschideCuva();
//                sleep(30000);
//            }
//            if (isStopRequested()) {
//                stop();
//            }
//        }
//
//        PoseStorage.currentPose = robot.drive.getPoseEstimate();
//
////        telemetry.addData("Status", "Run Time: " + runtime.toString());
//        telemetry.update();
//    }
//}
