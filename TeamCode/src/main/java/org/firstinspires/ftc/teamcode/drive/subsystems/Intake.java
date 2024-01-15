package org.firstinspires.ftc.teamcode.drive.subsystems;

import static java.lang.Math.abs;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Intake {

    public DcMotor motorSweeper, conveyor;
    public Servo servoGhearaStanga, servoGhearaDreapta;


    public Intake(HardwareMap hardwareMap){
        motorSweeper = hardwareMap.dcMotor.get("motorSweeper");
        conveyor = hardwareMap.dcMotor.get("conveyor");

        servoGhearaStanga = hardwareMap.servo.get("servoGhearaStanga");
        servoGhearaDreapta = hardwareMap.servo.get("servoGhearaDreapta");




        //Motor initialization
        motorSweeper.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorSweeper.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorSweeper.setDirection(DcMotorSimple.Direction.REVERSE);

        conveyor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        conveyor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        conveyor.setDirection(DcMotorSimple.Direction.REVERSE);

        servoGhearaStanga.setDirection(Servo.Direction.FORWARD);
        servoGhearaDreapta.setDirection(Servo.Direction.FORWARD);
    }

    public void setSweepPower(double pow){
        motorSweeper.setPower(pow);
    }
//    public void ridicaSweeper(){
//        servoSweeper.setPosition(0);
//    }

//    public void coboaraSweeper(){
//        servoSweeper.setPosition(1);
//    }

    public void activateConveyor(double pow){
        conveyor.setPower(pow);
    }
    public void stopConveyor(){
        conveyor.setPower(0);
    }

    public void inchideGhearapos(double pos){
        servoGhearaDreapta.setPosition(pos);
        servoGhearaStanga.setPosition(pos);
    }
    public void inchideGheara(){
        servoGhearaDreapta.setPosition(1);
        servoGhearaStanga.setPosition(1);
    }
    public void deschideGheara(){
        servoGhearaDreapta.setPosition(0.1);
        servoGhearaStanga.setPosition(0.2);
    }


}
