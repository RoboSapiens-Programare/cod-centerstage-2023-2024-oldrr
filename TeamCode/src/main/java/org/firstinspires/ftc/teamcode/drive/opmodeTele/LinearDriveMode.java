package org.firstinspires.ftc.teamcode.drive.opmodeTele;

import static java.lang.Math.abs;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.robot.MecanumRobot;
import org.firstinspires.ftc.teamcode.util.PoseStorage;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="MecanumDriveMode", group="Linear OpMode")

public class LinearDriveMode extends LinearOpMode {
    private MecanumRobot robot = null;
    boolean goingUp = false, goingDown = false;
    private double pos = 0.3;
    private double posStick = 0;
    private int pozIntake = 1;

    public double calculateThrottle(float x){
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

        while (robot.isInitialize() && opModeIsActive()) {
            idle();
        }

        telemetry.addData(">", "Initialized");
        telemetry.update();
        robot.intake.ridicaSweeper();
        robot.outtake.susCuva();
        robot.outtake.coboaraCuva();
        robot.fixerAndDrone.activateDrone();
        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()) {

        /**GAMEPAD 2**/
            if(gamepad2.left_stick_button && gamepad2.right_stick_button){
                robot.fixerAndDrone.releaseDrone();
            }

            if (gamepad2.left_trigger > 0.1) {
                robot.outtake.manualTarget = robot.outtake.motorGlisiera1.getCurrentPosition() - calculateThrottle(gamepad2.left_trigger * 12);
                robot.outtake.manualTarget--;
                robot.outtake.manualLevel(robot.outtake.manualTarget);
            }
            if (gamepad2.right_trigger > 0.1) {
                robot.outtake.manualTarget = robot.outtake.motorGlisiera1.getCurrentPosition() + calculateThrottle(gamepad2.right_trigger * 12);
                robot.outtake.manualTarget++;
                robot.outtake.manualLevel(robot.outtake.manualTarget);
            }

            if (gamepad2.dpad_down) {
                robot.outtake.manualLevel(Math.max(200, robot.outtake.motorGlisiera1.getCurrentPosition()));
                goingDown = true;
                robot.outtake.susCuva();
                robot.outtake.coboaraCuva();
            }

            if(goingDown && robot.outtake.outtakeLimit.isPressed())
            {
                robot.outtake.manualLevel(0);
                goingDown = false;
            }

            if (gamepad2.dpad_up) {
                robot.outtake.manualLevel(Math.max(200, robot.outtake.motorGlisiera1.getCurrentPosition()));
                goingUp = true;
            }

            if(goingUp && robot.outtake.motorGlisiera1.getCurrentPosition() >= 200){
                robot.outtake.ridicaCuva();
                robot.outtake.susCuva();
                goingUp = false;
            }

            if(gamepad2.cross){
                posStick = gamepad2.left_stick_x;
                if(posStick < 0) posStick = posStick / 2 + 0.5;
                robot.outtake.setCuva(posStick);
            }
            if(gamepad2.dpad_left){
                robot.outtake.stangaCuva();
            }
            if(gamepad2.dpad_right){
                robot.outtake.dreaptaCuva();
            }
            if(gamepad2.left_bumper){
                robot.outtake.servoCuvaGecko.setPower(-0.7);
            }
            else if(gamepad2.right_bumper){
                robot.outtake.servoCuvaGecko.setPower(1);
            }
            else robot.outtake.servoCuvaGecko.setPower(0);

            if (gamepad2.touchpad) {
                robot.outtake.resetMotors();
            }

        /** GAMEPAD1 **/
            if (gamepad1.right_bumper) {
                robot.intake.setSweepPower(1);

            }
            else if (gamepad1.left_bumper) {
                robot.intake.setSweepPower(-1);
            }
            else if (gamepad1.right_trigger > 0.1) {
                robot.intake.setSweepPower(calculateThrottle(gamepad1.right_trigger));

            } else if (gamepad1.left_trigger > 0.1) {
                robot.intake.setSweepPower(calculateThrottle(-gamepad1.left_trigger));

            } else {
                robot.intake.setSweepPower(0);
            }

            if(gamepad1.dpad_down){
                pos = pos + 0.01;
                sleep(250);
            }
            if(gamepad1.dpad_up){
                pos = pos - 0.01;
                sleep(250);
            }
            if(gamepad1.dpad_right){
                robot.intake.setSweeper(pos);
            }
//
//            if (gamepad1.cross) {
//                robot.hanger.motorHanger.setPower(1);
//                robot.hanger.hangerLock.setPosition(1);
//            } else if (gamepad1.circle) {
//                robot.hanger.motorHanger.setPower(-1);
//                robot.hanger.hangerLock.setPosition(1);
//            } else {
//                robot.hanger.motorHanger.setPower(0);
//            }
//
//
////            if(gamepad1.square) {
////                robot.intake.servoGhearaStanga.setPosition(0.5);
////                robot.intake.servoGhearaDreapta.setPosition(0.5);
////            }
////            if(gamepad1.touchpad)
////                robot.intake.inchideGheara();
////            if(gamepad1.triangle) robot.intake.deschideGheara();

            robot.drive.update();

//            poseEstimate = robot.drive.getPoseEstimate();

            double backboardMultiplier = 1;
//            if(poseEstimate.getY())
            robot.drive.setDrivePower(new Pose2d(calculateThrottle((-gamepad1.left_stick_y)), calculateThrottle((float) (-gamepad1.left_stick_x)), calculateThrottle((float) (-gamepad1.right_stick_x))));


            /** TELEMETRY **/

//            telemetry.addData("AutoMode: ", autoMode);
            telemetry.addData("SERVO POS: ", pos);
            telemetry.addData("OUTTAKE LIMIT: ", robot.outtake.outtakeLimit.isPressed());
//            telemetry.addData("LOWER LIMIT: ", robot.outtake.lowerLimit.isPressed());
            telemetry.addData("Slide ticks: ", robot.outtake.motorGlisiera1.getCurrentPosition());
            telemetry.addData("Slide ticks2: ", robot.outtake.motorGlisiera2.isBusy());

            telemetry.update();
        }
    }
}