//package org.firstinspires.ftc.teamcode.drive.opmodeTele;
//
//import com.acmerobotics.roadrunner.control.PIDFController;
//import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.hardware.IMU;
//import com.acmerobotics.roadrunner.control.PIDCoefficients;
//import com.acmerobotics.dashboard.config.Config;
//
//import org.firstinspires.ftc.teamcode.drive.robot.MecanumRobot;
//import org.firstinspires.ftc.teamcode.drive.subsystems.BalancingRobot;
//@Config
//@TeleOp(name="MecanumDriveMode", group="Linear OpMode")
//public class BalancingBoy extends LinearOpMode{
//    private BalancingRobot robot = null;
//    public double kd=0,kp=0.01,kf=0;
//    private static  PIDCoefficients pidc = new PIDCoefficients(kp,kd,kf);
//    PIDFController pid = new PIDFController(pidc);
//
//    @Override
//    public void runOpMode() throws InterruptedException {
//        telemetry.addData(">", "Initializing...");
//        telemetry.update();
//
//        robot = new BalancingRobot(hardwareMap);
//        robot.motorDreapta.setPower(1);
//        robot.IMU.initialize(
//                new IMU.Parameters(
//                        new RevHubOrientationOnRobot(
//                                RevHubOrientationOnRobot.LogoFacingDirection.UP,
//                                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
//                        )
//                )
//        );
//        while(opModeIsActive())
//        {
//
//        }
//}}
