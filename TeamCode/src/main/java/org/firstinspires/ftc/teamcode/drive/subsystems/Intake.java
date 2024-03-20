package org.firstinspires.ftc.teamcode.drive.subsystems;

import static java.lang.Math.abs;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Intake {
    public DcMotor motorSweeper;
    public Servo servoIntake;
    public Intake(HardwareMap hardwareMap) {
        motorSweeper = hardwareMap.dcMotor.get("motorSweeper");

        servoIntake = hardwareMap.servo.get("servoIntake");

        //Motor initialization
        motorSweeper.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorSweeper.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorSweeper.setDirection(DcMotorSimple.Direction.FORWARD);

        servoIntake.setDirection(Servo.Direction.FORWARD);
    }
    public void setSweepPower ( double pow){
        motorSweeper.setPower(pow);
    }
    public void ridicaSweeper(){
        servoIntake.setPosition(0);
    }

    public void coboaraSweeper(){
        servoIntake.setPosition(1);
    }

    public void setSweeper(double pos){
        servoIntake.setPosition(pos);
    }
}