package org.firstinspires.ftc.teamcode.drive.opmodeTele;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static java.lang.Math.abs;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
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
    private double pos = 0.4965, pos1 = 0, pos2 = 0.5;

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
        //INIT CODE
        AnalogInput analogInput = hardwareMap.get(AnalogInput.class, "analogInput");
        telemetry.addData(">", "Initialized");
        telemetry.update();
//        robot.intake.ridicaSweeper();
        robot.outtake.servoRotCuva.setPosition(pos);
        robot.outtake.coboaraCuva();
        robot.fixerAndDrone.activateDrone();
        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()) {
        //TELEOP CODE
        /**GAMEPAD 2**/
            double positionAbso = analogInput.getVoltage() / 3.3 * 360;
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
                robot.outtake.manualLevel(300, 0.6);
                goingDown = true;
                robot.outtake.servoRotCuva.setPosition(0.5);
                robot.outtake.coboaraCuva();
            }

            if(goingDown && positionAbso >= 280)
            {
                robot.outtake.manualLevel(-20, 0.5);
//                robot.outtake.servoStanga.getController().pwmDisable();
                goingDown = false;
            }

            if (gamepad2.dpad_up) {
                robot.outtake.manualLevel(Math.max(200, robot.outtake.motorGlisiera1.getCurrentPosition()));
                goingUp = true;
            }

            if(goingUp && robot.outtake.motorGlisiera1.getCurrentPosition() >= 150){
                robot.outtake.ridicaCuva();
                pos = 0.4965;
                goingUp = false;
            }

            if(Math.abs(gamepad2.left_stick_x) >= 0.1 && robot.outtake.motorGlisiera1.getCurrentPosition() >= 175)
                pos += 0.025 * gamepad2.left_stick_x;
            if(Math.abs(gamepad2.right_stick_y) >= 0.1)
                pos1 += 0.025 * gamepad2.right_stick_y;

//            if(gamepad2.dpad_left){
//                pos = 0.15;
//            }
//
//            if(gamepad2.dpad_right){
//                pos = 0.85;
//            }

            if(pos < 0.15)
                pos = 0.15;
            else if(pos > 0.85)
                pos = 0.85;

            if(pos1 < 0)
                pos1 = 0;
            else if(pos1 > 0.4)
                pos1 = 0.4;

            robot.outtake.servoRotCuva.setPosition(pos);
            robot.intake.servoIntake.setPosition(pos1);

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
                robot.intake.ridicaSweeper();
            }
            if(gamepad1.dpad_up){
                robot.intake.coboaraSweeper();
            }

//            if(Math.abs(gamepad2.right_stick_x) >= 0.1){
//                pos1 += 0.025 * gamepad2.right_stick_x;
//            }
//
//            if(Math.abs(gamepad2.right_stick_y) >= 0.1){
//                pos2 += 0.025 * gamepad2.right_stick_y;
//            }
//
//            robot.intake.servoIntake.setPosition(pos1);
//            robot.fixerAndDrone.servoAvion.setPosition(pos2);

            robot.drive.update();

//            poseEstimate = robot.drive.getPoseEstimate();

            double backboardMultiplier = 1;
//            if(poseEstimate.getY())
            robot.drive.setDrivePower(new Pose2d(calculateThrottle((-gamepad1.left_stick_y)) * 0.8, calculateThrottle((float) (-gamepad1.left_stick_x)) * 0.8, calculateThrottle((float) (-gamepad1.right_stick_x)) * 0.8));


            /** TELEMETRY **/

//            telemetry.addData("AutoMode: ", autoMode);
            telemetry.addData("SERVO POS: ", pos);
            telemetry.addData("OUTTAKE LIMIT: ", robot.outtake.outtakeLimit.isPressed());
//            telemetry.addData("LOWER LIMIT: ", robot.outtake.lowerLimit.isPressed());
            telemetry.addData("Slide ticks: ", robot.outtake.motorGlisiera1.getCurrentPosition());
            telemetry.addData("Slide ticks2: ", robot.outtake.motorGlisiera2.isBusy());
            telemetry.addData("Extensie: ", pos1);
            telemetry.addData("avion, ", pos2);
            telemetry.addData("ServoPosAbso: ", positionAbso);

            telemetry.update();
        }
    }
}