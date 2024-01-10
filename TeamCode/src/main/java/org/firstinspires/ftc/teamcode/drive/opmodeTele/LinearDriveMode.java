package org.firstinspires.ftc.teamcode.drive.opmodeTele;

import static java.lang.Math.abs;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drive.robot.MecanumRobot;
import org.firstinspires.ftc.teamcode.drive.robot.StandardTrackingWheelLocalizer;
import org.firstinspires.ftc.teamcode.util.PoseStorage;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

@TeleOp(name="MecanumDriveMode", group="Linear OpMode")

public class LinearDriveMode extends LinearOpMode {
    private MecanumRobot robot = null;
    boolean subZero = false, autoMode = true;
    private double pos = 0.3;
    private ElapsedTime timer;
    ColorSensor color;
    boolean changed = false; //Outside of loop()
    Pose2d poseEstimate;

    //    private DcMotor hangerMotor = null, winchMotor = null;
    public double calculateThrottle(float x) {
        int sign = -1;
        if (x > 0) sign = 1;
        return sign * Math.pow(100 * (abs(x) / 100), 2);
    }

    @Override
    public void runOpMode() throws InterruptedException {

        telemetry.addData(">", "Initializing...");
        telemetry.update();

        robot = new MecanumRobot(hardwareMap);
        robot.drive.setPoseEstimate(PoseStorage.currentPose);

//        color = hardwareMap.get(ColorSensor.class, "Color");
        while (robot.isInitialize() && opModeIsActive()) {
            idle();
        }

//        robot.camera.visionPortal.resumeStreaming();

        telemetry.addData(">", "Initialized");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()) {
            if (gamepad2.left_trigger > 0.1) {
                robot.outtake.manualTarget = robot.outtake.motorGlisiera.getCurrentPosition() - calculateThrottle(gamepad2.left_trigger * 12);
                robot.outtake.manualTarget--;
                robot.outtake.manualLevel(robot.outtake.manualTarget);
            }
            if (gamepad2.right_trigger > 0.1) {
                robot.outtake.manualTarget = robot.outtake.motorGlisiera.getCurrentPosition() + calculateThrottle(gamepad2.right_trigger * 12);
                robot.outtake.manualTarget++;
                robot.outtake.manualLevel(robot.outtake.manualTarget);
            }

            if (gamepad2.share) {
                autoMode = false;
            }
            if (gamepad2.options) {
                autoMode = true;
            }

            if (autoMode) {
                if (robot.outtake.pixelStanga()) {
                    robot.outtake.inchideStanga();
                }
                if (robot.outtake.pixelDreapta()) {
                    robot.outtake.inchideDreapta();
                }
                if (!robot.outtake.pixelStanga()) {
                    robot.outtake.deschideStanga();
                }
                if (!robot.outtake.pixelDreapta()) {
                    robot.outtake.deschideDreapta();
                }
                if (gamepad2.square) {
                    robot.outtake.inchideCuva();
                    robot.outtake.manualLevel(700);
                    if (robot.outtake.motorGlisiera.getCurrentPosition() >= 400) {
                        robot.outtake.ridicaCuva();
                        autoMode = false;
                    }
                }
                if (gamepad2.triangle) {
                    robot.outtake.inchideCuva();
                    robot.outtake.coboaraCuva();
                    timer = new ElapsedTime();
                    timer.startTime();
                    if(timer.milliseconds() > 200) {
                        robot.outtake.manualLevel(-50);
                        timer.reset();
                    }
                    if (robot.outtake.motorGlisiera.getCurrentPosition() <= 50) {
                        robot.outtake.deschideCuva();
                    }
                    if (!robot.outtake.motorGlisiera.isBusy()) {
                        autoMode = true;
                    }
                }

            } else {

                if (gamepad2.triangle) {
                    robot.outtake.inchideCuva();
                    robot.outtake.coboaraCuva();
                    timer = new ElapsedTime();
                    timer.startTime();
                    if(timer.milliseconds() > 200) {
                        robot.outtake.manualLevel(-50);
                        timer.reset();
                    }
                    //autoMode = true;
                    if (robot.outtake.motorGlisiera.getCurrentPosition() <= 50) {
                        robot.outtake.deschideCuva();
                    }
                    if (!robot.outtake.motorGlisiera.isBusy()) autoMode = true;
                }

                if (gamepad2.square) {
                    robot.outtake.inchideCuva();
                    robot.outtake.manualLevel(700);
                    if (robot.outtake.motorGlisiera.getCurrentPosition() >= 400) {
                        robot.outtake.ridicaCuva();
                        autoMode = false;
                    }
                }

                if (gamepad2.left_bumper) {
                    robot.outtake.deschideCuva();
                }

                if (gamepad2.right_bumper) {
                    robot.outtake.inchideCuva();
                }
            }

            double pozitieGlisiera = robot.outtake.motorGlisiera.getCurrentPosition();
            if (gamepad2.dpad_up) {
                if(pozitieGlisiera > 700 - 50 && pozitieGlisiera <= 900 - 50)
                    robot.outtake.manualLevel(900);
                else if(pozitieGlisiera > 900 - 50 && pozitieGlisiera <= 1200 - 50)
                    robot.outtake.manualLevel(1200);
                else
                    robot.outtake.manualLevel(1400);
            }

            if (gamepad2.dpad_down) {
                if(pozitieGlisiera > 900 - 50 && pozitieGlisiera <= 1200 - 50)
                    robot.outtake.manualLevel(700);
                else if(pozitieGlisiera > 1200 - 50 && pozitieGlisiera <= 1400 - 50)
                    robot.outtake.manualLevel(900);
                else robot.outtake.manualLevel(1200);
            }

//                if(gamepad2.dpad_up){
//                    pos += 0.05;
//                    timer.startTime();
//                    timer = new ElapsedTime(250);
//                }
//                if(gamepad2.dpad_down){
//                    pos -= 0.05;
//                    timer.startTime();
//                    timer = new ElapsedTime(250);
//                }
//
//                if(gamepad2.dpad_right)
//                    robot.outtake.setCuva(pos);

            if (gamepad2.cross) {
                robot.outtake.ridicaCuva();
            }
            if (gamepad2.circle) {
                robot.outtake.coboaraCuva();
            }
            if (gamepad2.touchpad) {
                robot.outtake.manualLevel(robot.outtake.motorGlisiera.getCurrentPosition() + 30);
            }


            /** GAMEPAD1 **/

            if (gamepad1.right_bumper) {
                robot.intake.setSweepPower(0.4);
                robot.intake.activateConveyor(-1);
            } else if (gamepad1.left_bumper) {
                robot.intake.setSweepPower(-0.4);
                robot.intake.activateConveyor(1);
            } else {
                robot.intake.setSweepPower(0);
                robot.intake.stopConveyor();
            }

            if (gamepad1.cross) {
                robot.hanger.motorHanger.setPower(1);
            } else if (gamepad1.circle) {
                robot.hanger.motorHanger.setPower(-1);
            } else {
                robot.hanger.motorHanger.setPower(0);
            }

            robot.drive.update();

            poseEstimate = robot.drive.getPoseEstimate();

            double backboardMultiplier = 1;
//            if(poseEstimate.getY())
            robot.drive.setDrivePower(new Pose2d(calculateThrottle((-gamepad1.left_stick_y)), calculateThrottle((float) (-gamepad1.left_stick_x)), calculateThrottle((float) (-gamepad1.right_stick_x))));


            /** TELEMETRY **/

            telemetry.addData("AutoMode: ", autoMode);
            telemetry.addData("Slide ticks: ", robot.outtake.motorGlisiera.getCurrentPosition());
            telemetry.addData("Sweeper power: ", robot.intake.motorSweeper.getPower());
            telemetry.addData("Servopos: ", pos);
            telemetry.addData("senzorDistanta1: ", robot.outtake.senzorDistanta1.getDistance(DistanceUnit.CM));
            telemetry.addData("senzorDistanta2: ", robot.outtake.senzorDistanta2.getDistance(DistanceUnit.CM));

            if (robot.outtake.pixelStanga() && robot.outtake.pixelDreapta()) {
                telemetry.addLine("Both Pixels engaged!");
            } else if (robot.outtake.pixelStanga()) {
                telemetry.addLine("Left Pixel engaged!");
            } else if (robot.outtake.pixelDreapta()) {
                telemetry.addLine("Right Pixel engaged!");
            }

//            robot.camera.telemetryAprilTag(telemetry);

            telemetry.addData("x: ", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("Heading: ", poseEstimate.getHeading());

            telemetry.update();
        }
    }
}

