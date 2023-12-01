package org.firstinspires.ftc.teamcode.drive.robot;


import com.acmerobotics.roadrunner.drive.MecanumDrive;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.drive.subsystems.Intake;
import org.firstinspires.ftc.teamcode.drive.robot.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.subsystems.Outtake;


public class MecanumRobot {
    private boolean initialize;
    public SampleMecanumDrive drive;
    public Intake intake;
    public Outtake outtake;


    public MecanumRobot(HardwareMap hardwareMap){
        initialize = true;
        intake = new Intake(hardwareMap);
        outtake = new Outtake(hardwareMap);
        drive = new SampleMecanumDrive(hardwareMap);
        initialize = false;
    }
    public boolean isInitialize() {return initialize;}
}