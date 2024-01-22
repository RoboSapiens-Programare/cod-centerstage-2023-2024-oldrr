package org.firstinspires.ftc.teamcode.drive.subsystems;

import static java.lang.Math.abs;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;




public class Hanger {
    private static final double POWER = 1;
    public DcMotor motorHanger;
    public Servo hangerLock;

    public Hanger(HardwareMap hardwareMap){
        motorHanger = hardwareMap.dcMotor.get("motorHanger");
        hangerLock = hardwareMap.servo.get("servoHanger");

        //Motor initialization
        motorHanger.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorHanger.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorHanger.setDirection(DcMotorSimple.Direction.FORWARD);

        hangerLock.setDirection(Servo.Direction.FORWARD);
        hangerLock.setPosition(0);
    }
}


