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
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.robot.Robot;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.robot.MecanumRobot;
import org.firstinspires.ftc.teamcode.drive.vision.OpenCVThread;
import org.firstinspires.ftc.teamcode.drive.vision.PiramidaAlbastru;
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

@Autonomous(name = "Autonomie roadrunner parcare albastru", group="autonomous")

public class Autonomiealbastruaproape extends LinearOpMode {
//    Declare OpMode members.
//    private ElapsedTime runtime = new ElapsedTime();
    private MecanumRobot robot = null;
    private ElapsedTime timer;
    public OpenCVThread openCV;
    public ElapsedTime opencvTimer;
    public static int MAX_MILISECONDS = 5000;
    private PiramidaAlbastru.Location finalLocation;


    public void initAutonomous() {
        telemetry.addData(">", "Initializing...");
        telemetry.update();
        robot = new MecanumRobot(hardwareMap);

        openCV = new OpenCVThread(hardwareMap);
        finalLocation = PiramidaAlbastru.Location.RIGHT;

        openCV.start();

        timer = new ElapsedTime();
        telemetry.addData("has initialised", "yes");
        telemetry.update();
        while (robot.isInitialize() && opModeIsActive()) {
            idle();
        }

        telemetry.addData(">", "Initialized");
        telemetry.update();
    }

    @Override
    public void runOpMode() throws InterruptedException {
        initAutonomous();

        opencvTimer = new ElapsedTime();
        opencvTimer.startTime();
        timer.startTime();

        while (opencvTimer.milliseconds() < MAX_MILISECONDS) {
            telemetry.addData("Location: ", openCV.getLocation());
            telemetry.update();
            finalLocation = openCV.getLocation();
        }

//        try {
//            openCV.finalize();
//        } catch (Throwable throwable) {
//            throwable.printStackTrace();
//        }

        telemetry.setMsTransmissionInterval(50);


        /*
         * The INIT-loop:
         * This REPLACES waitForStart!
         */

        while (!isStarted() && !isStopRequested()) {

        }

        if (isStopRequested()) return;

        waitForStart();

        while (opModeIsActive()) {
            robot.outtake.inchideCuva();
            Pose2d start = new Pose2d(35, -60, Math.toRadians(90));
            robot.drive.setPoseEstimate(start);
            TrajectorySequence myTrajectory1 = robot.drive.trajectorySequenceBuilder(start)
                .forward(24)
                .turn(Math.toRadians(-90))
                .back(32)
                .addTemporalMarker(() -> {
                          robot.outtake.manualLevel(1230);
                          robot.outtake.inchideCuva();
                          sleep(1000);
                          robot.outtake.ridicaCuva();
                })
                .back(5)
                .waitSeconds(0.3)
                .addTemporalMarker(() -> {
                  robot.outtake.deschideCuva();
                })
                .waitSeconds(1)
                .forward(7)
                .addTemporalMarker(() -> {
                  robot.outtake.inchideCuva();
                  robot.outtake.coboaraCuva();
                  robot.outtake.manualLevel(-100);
                  })
                .strafeRight(24)
                .back(17)
                .build();
            robot.drive.followTrajectorySequence(myTrajectory1);
            sleep(30000);
        }

        PoseStorage.currentPose = robot.drive.getPoseEstimate();

        stop();
//        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.update();
    }
}
