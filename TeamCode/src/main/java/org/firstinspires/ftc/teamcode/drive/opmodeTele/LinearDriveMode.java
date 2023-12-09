package org.firstinspires.ftc.teamcode.drive.opmodeTele;

import static java.lang.Math.abs;

import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.firstinspires.ftc.teamcode.drive.robot.MecanumRobot;

@TeleOp(name="MecanumDriveMode", group="Linear OpMode")

public class LinearDriveMode extends LinearOpMode {
    private MecanumRobot robot = null;
    boolean subZero = false;
    private double pos = 0.3;
    ColorSensor color;
    boolean changed = false; //Outside of loop()

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


//        color = hardwareMap.get(ColorSensor.class, "Color");
        while (robot.isInitialize() && opModeIsActive()) {
            idle();
        }

        telemetry.addData(">", "Initialized");
        telemetry.update();


        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()) {
            Pose2d poseEstimate = robot.drive.getPoseEstimate();

            // Create a vector from the gamepad x/y inputs
            // Then, rotate that vector by the inverse of that heading
            Vector2d input = new Vector2d(
                    -gamepad1.left_stick_y,
                    -gamepad1.left_stick_x
            ).rotated(-poseEstimate.getHeading());

            // Pass in the rotated input + right stick value for rotation
            // Rotation is not part of the rotated input thus must be passed in separately
            robot.drive.setWeightedDrivePower(
                    new Pose2d(
                            input.getX(),
                            input.getY(),
                            -gamepad1.right_stick_x
                    )
            );

            // Update everything. Odometry. Etc.
            robot.drive.update();


            if (gamepad2.right_trigger > 0.1) {
                robot.outtake.manualTarget = robot.outtake.motorGlisiera.getCurrentPosition() - calculateThrottle(gamepad2.right_trigger * 12);
                robot.outtake.manualTarget--;
                robot.outtake.manualLevel(robot.outtake.manualTarget);
            }
            if(robot.outtake.motorGlisiera.getCurrentPosition() > 1 ){
                subZero = true;
            }
            if(robot.outtake.motorGlisiera.getCurrentPosition() <= -10){
                subZero = false;
            }
            if (gamepad2.left_trigger > 0.1 && !subZero) {
                robot.outtake.manualTarget = robot.outtake.motorGlisiera.getCurrentPosition() + calculateThrottle(gamepad2.left_trigger * 12);
                robot.outtake.manualTarget++;
                robot.outtake.manualLevel(robot.outtake.manualTarget);
            }


//            if(robot.outtake.motorGlisiera.getCurrentPosition() > 100 && robot.outtake.motorGlisiera.getCurrentPosition() < 255){
//                robot.outtake.inchideCuva();
//                robot.outtake.servoStanga.setPosition(0);
//                robot.outtake.servoDreapta.setPosition(0);
//            }
//            if(robot.outtake.motorGlisiera.getCurrentPosition() > 254 && robot.outtake.motorGlisiera.getCurrentPosition() < 400){
//                robot.outtake.inchideCuva();
//                robot.outtake.servoStanga.setPosition(0.05);
//                robot.outtake.servoDreapta.setPosition(0.05);
//            }

//            if(gamepad2.left_stick_button && !changed) {
//                if(robot.intake.servoSweeper.getPosition() == 0.1){
//                    robot.intake.servoSweeper.setPosition(0.8);
//                }
//                else {
//                    robot.intake.servoSweeper.setPosition(0.1);
//                }
//                changed = true;
//            }
//            else if(!gamepad2.left_stick_button) changed = false;

            if(gamepad2.triangle){
                robot.outtake.inchideCuva();
                robot.outtake.coboaraCuva();
                robot.outtake.manualLevel(0);
                if(robot.outtake.motorGlisiera.getCurrentPosition() >= -5 && robot.outtake.motorGlisiera.getCurrentPosition() <= 5){
                    robot.outtake.deschideCuva();
                }
            }
            if(gamepad2.square){
                robot.outtake.inchideCuva();
                robot.outtake.manualLevel(-1230);
                if(robot.outtake.motorGlisiera.getCurrentPosition() >= -1235 && robot.outtake.motorGlisiera.getCurrentPosition() <= -1225){
                    robot.outtake.ridicaCuva();
                }
            }

            if(gamepad2.left_bumper){
                robot.outtake.deschideCuva();
            }

            if(gamepad2.right_bumper){
                robot.outtake.inchideCuva();
            }

            if(gamepad2.cross){
                robot.outtake.ridicaCuva();
            }
            if(gamepad2.circle){
                robot.outtake.coboaraCuva();
            }



            /** GAMEPAD1 **/

            if(gamepad1.right_bumper) {
                robot.intake.setSweepPower(0.4);
                robot.intake.activateConveyor(1);
            }
            else if(gamepad1.left_bumper) {
                robot.intake.setSweepPower(-0.4);
                robot.intake.activateConveyor(-1);
            }
            else {
                robot.intake.setSweepPower(0);
                robot.intake.stopConveyor();
            }

//            if(gamepad1.cross){
//                robot.intake.setSweepPower(0.4);
//            }
//            else if(gamepad1.triangle){
//                robot.intake.setSweepPower(-0.4);
//            }
//            else robot.intake.setSweepPower(0);
//            if(gamepad1.dpad_down){
//                robot.intake.activateConveyor(1);
//            }
//            else if(gamepad1.dpad_up){
//                robot.intake.activateConveyor(-1);
//            }
//            else robot.intake.stopConveyor();



//            robot.drive.setDrivePower(new Pose2d(calculateThrottle((-gamepad1.left_stick_y)), calculateThrottle((float) (-gamepad1.left_stick_x)), calculateThrottle((float) (-gamepad1.right_stick_x))));
            telemetry.addData("Slide ticks: ", robot.outtake.motorGlisiera.getCurrentPosition());
            telemetry.addData("Sweeper power: ", robot.intake.motorSweeper.getPower());
            telemetry.addData("Servopos: ", pos);
            telemetry.update();
        }

    }
}
