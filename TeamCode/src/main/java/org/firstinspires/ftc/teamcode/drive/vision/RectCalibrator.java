package org.firstinspires.ftc.teamcode.drive.vision;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.robot.MecanumRobot;
import org.opencv.core.Rect;

@TeleOp(name = "Rectangle calibrator", group = "Linear OpMode")
public class RectCalibrator extends LinearOpMode {

    public OpenCVThreadRosuAproape openCV;
    public ElapsedTime opencvTimer;

    int x1 = 0, y1 = 0, x2 = 0, y2 = 0;
    public static int MAX_MILISECONDS = 5000;
    private PiramidaRosuAproape.Location finalLocation;


    public void runOpMode() {
        telemetry.addData(">", "Initializing...");
        telemetry.update();

        openCV = new OpenCVThreadRosuAproape(hardwareMap);
        finalLocation = PiramidaRosuAproape.Location.LEFT;

        openCV.start();

        telemetry.addData("has initialised", "yes");
        telemetry.update();
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

        while(opModeIsActive()){
            if(gamepad1.dpad_left) {
                x1--;
                sleep(250);
            }
            if(gamepad1.dpad_right){
                x1++;
                sleep(250);
            }
            if(gamepad1.square) {
                x2--;
                sleep(250);
            }
            if(gamepad1.circle) {
                x2++;
                sleep(250);
            }

            if(gamepad1.dpad_up) {
                y1--;
                sleep(250);
            }
            if(gamepad1.dpad_down) {
                y1++;
                sleep(250);
            }
            if(gamepad1.triangle) {
                y2--;
                sleep(250);
            }

            if(gamepad1.cross) {
                y2++;
                sleep(250);
            }

            telemetry.addData("x1 ", x1);
            telemetry.addData("y1 ", y1);
            telemetry.addData("x2 ", x2);
            telemetry.addData("y2 ", y2);

            if(gamepad1.left_bumper)
                PiramidaRosuAproape.CENTER_ROI = new Rect(x1, y1, x2, y2);
            if(gamepad1.right_bumper);
                PiramidaRosuAproape.RIGHT_ROI = new Rect(x1, y1, x2, y2);
        }
    }
}