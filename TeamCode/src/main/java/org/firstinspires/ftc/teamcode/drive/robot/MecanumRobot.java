package org.firstinspires.ftc.teamcode.drive.robot;


import android.graphics.Camera;

import com.acmerobotics.roadrunner.drive.MecanumDrive;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

//import org.firstinspires.ftc.teamcode.drive.subsystems.AprilTagCamera;
//import org.firstinspires.ftc.teamcode.drive.subsystems.Hanger;
//import org.firstinspires.ftc.teamcode.drive.subsystems.Intake;
import org.firstinspires.ftc.teamcode.drive.robot.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.subsystems.FixerAndDrone;
import org.firstinspires.ftc.teamcode.drive.subsystems.Intake;
import org.firstinspires.ftc.teamcode.drive.subsystems.Outtake;
//import org.firstinspires.ftc.teamcode.drive.subsystems.Outtake;


public class MecanumRobot {
    private boolean initialize;
    public SampleMecanumDrive drive;
    public Intake intake;
    public Outtake outtake;
    public FixerAndDrone fixerAndDrone;
//    public Hanger hanger;
//    public AprilTagCamera camera;

    public MecanumRobot(HardwareMap hardwareMap){
        initialize = true;
        drive = new SampleMecanumDrive(hardwareMap);
        intake = new Intake(hardwareMap);
        outtake = new Outtake(hardwareMap);
        fixerAndDrone = new FixerAndDrone(hardwareMap);
//        camera = new AprilTagCamera(hardwareMap);
        initialize = false;

        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    public boolean isInitialize() {return initialize;}
}