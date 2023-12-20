package org.firstinspires.ftc.teamcode.drive.opmodeTele;

import static java.lang.Math.abs;
import static java.lang.System.currentTimeMillis;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drive.robot.MecanumRobot;

@TeleOp(name="MecanumDriveModeGeorge", group="Linear OpMode")

public class LinearDriveModeGeorge extends LinearOpMode {
    private MecanumRobot robot = null;
    boolean subZero = false, autoMode = false;
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

            if(robot.outtake.pixelStanga() && robot.outtake.pixelDreapta()){
                telemetry.addLine("Both Pixels engaged!");
            }
            else if(robot.outtake.pixelStanga()){
                telemetry.addLine("Left Pixel engaged!");
            }
            else if(robot.outtake.pixelDreapta()){
                telemetry.addLine("Right Pixel engaged!");
            }



//            if (gamepad2.left_trigger > 0.1) {
//                robot.outtake.manualTarget = robot.outtake.motorGlisiera.getCurrentPosition() - calculateThrottle(gamepad2.left_trigger * 12);
//                robot.outtake.manualTarget--;
//                robot.outtake.manualLevel(robot.outtake.manualTarget);
//            }
//            if(robot.outtake.motorGlisiera.getCurrentPosition() > 1 ){
//                subZero = true;
//            }
//            if(robot.outtake.motorGlisiera.getCurrentPosition() <= -10){
//                subZero = false;
//            }
//            if (gamepad2.right_trigger > 0.1 && !subZero) {
//                robot.outtake.manualTarget = robot.outtake.motorGlisiera.getCurrentPosition() + calculateThrottle(gamepad2.right_trigger * 12);
//                robot.outtake.manualTarget++;
//                robot.outtake.manualLevel(robot.outtake.manualTarget);
//            }

            if(gamepad2.right_trigger > 0.1){
                robot.outtake.motorGlisiera.setPower(gamepad2.right_trigger);
            }
            else if(gamepad2.left_trigger > 0.1){
                robot.outtake.motorGlisiera.setPower(-gamepad2.left_trigger);
            }
            else robot.outtake.motorGlisiera.setPower(0);

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
            if(gamepad2.share){
                autoMode = false;
            }
            if(gamepad2.options){
                autoMode = true;
            }

            if(autoMode){
                if(robot.outtake.pixelStanga()){
                    robot.outtake.inchideStanga();
                }
                if(robot.outtake.pixelDreapta()){
                    robot.outtake.inchideDreapta();
                }
            }
            else {
                if (gamepad2.left_bumper) {
                    robot.outtake.deschideCuva();
                }
                if (gamepad2.right_bumper) {
                    robot.outtake.inchideCuva();
               }
//                if(gamepad2.dpad_up) {
//                    pos = pos + 0.01;
//                    sleep(200);
//                }
//                if(gamepad2.dpad_down){
//                    pos = pos - 0.01;
//                    sleep(200);
//                }
//                if(gamepad2.dpad_right){
//                    robot.outtake.servoDreapta.setPosition(pos);
//                    robot.outtake.servoStanga.setPosition(pos);


                if (gamepad2.dpad_left) {
                    robot.outtake.deschideStanga();
                }
                if (gamepad2.dpad_right) {
                    robot.outtake.deschideDreapta();
                }
                if (gamepad2.dpad_up) {
                    robot.outtake.inchideStanga();
                }
                if (gamepad2.dpad_down) {
                    robot.outtake.inchideDreapta();
                }
            }
            if (gamepad2.cross) {
                robot.outtake.ridicaCuva();
            }
            if (gamepad2.circle) {
                robot.outtake.coboaraCuva();
                robot.outtake.inchideCuva();
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

            robot.drive.setDrivePower(new Pose2d(calculateThrottle((-gamepad1.left_stick_y)), calculateThrottle((float) (-gamepad1.left_stick_x)), calculateThrottle((float) (-gamepad1.right_stick_x))));

            telemetry.addData("AutoMode: ", autoMode);
            telemetry.addData("Slide ticks: ", robot.outtake.motorGlisiera.getCurrentPosition());
            telemetry.addData("Sweeper power: ", robot.intake.motorSweeper.getPower());
            telemetry.addData("Servopos: ", pos);
            telemetry.addData("senzorDistanta1: ", robot.outtake.senzorDistanta1.getDistance(DistanceUnit.CM));
            telemetry.addData("senzorDistanta2: ", robot.outtake.senzorDistanta2.getDistance(DistanceUnit.CM));
            telemetry.update();
        }

    }
}
