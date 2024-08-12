package org.firstinspires.ftc.teamcode.drive.subsystems;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static java.lang.Math.abs;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

public class Outtake {
    private static final double POWER = 1;

    public DcMotor motorGlisiera1, motorGlisiera2;
    public TouchSensor outtakeLimit;
    public Servo servoStanga, servoDreapta, servoRotCuva;
    public CRServo servoCuvaGecko;
    public double manualTarget = 0;

    public Outtake(HardwareMap hardwareMap){
        motorGlisiera1 = hardwareMap.dcMotor.get("motorGlisiera");
        motorGlisiera2 = hardwareMap.dcMotor.get("motorGlisiera2");

        servoStanga = hardwareMap.servo.get("servoStanga");
        servoDreapta = hardwareMap.servo.get("servoDreapta");
        servoRotCuva = hardwareMap.servo.get("servoRotCuva");
        servoCuvaGecko = hardwareMap.crservo.get("ServoCuvaGecko");

//        lowerLimit = hardwareMap.touchSensor.get("lowerLimit");
        outtakeLimit = hardwareMap.touchSensor.get("outtakeLimit");

        //Motor initialization
        motorGlisiera1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorGlisiera1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorGlisiera1.setDirection(DcMotorSimple.Direction.FORWARD);
        motorGlisiera2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorGlisiera2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorGlisiera2.setDirection(DcMotorSimple.Direction.REVERSE);

        servoStanga.setDirection(Servo.Direction.REVERSE);
        servoDreapta.setDirection(Servo.Direction.FORWARD);
        servoRotCuva.setDirection(Servo.Direction.FORWARD);
        servoCuvaGecko.setDirection(DcMotorSimple.Direction.FORWARD);
//        servoCuvaStanga.setDirection(Servo.Direction.REVERSE);
    }


    public double calculateThrottle(float x) {
        int sign = -1;
        if (x > 0) sign = 1;
        return sign * Math.pow(100 * (abs(x) / 100), 2);
    }


    public void manualLevel(double manualTarget) {
        motorGlisiera1.setTargetPosition((int) manualTarget);
        motorGlisiera2.setTargetPosition((int) manualTarget);
        motorGlisiera1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorGlisiera2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        if(motorGlisiera1.getCurrentPosition() < manualTarget )
        {
            motorGlisiera1.setPower(POWER);
        }
        else{
            motorGlisiera1.setPower(-POWER);
        }
        if(motorGlisiera2.getCurrentPosition() < manualTarget )
        {
            motorGlisiera2.setPower(-POWER);
        }
        else{
            motorGlisiera2.setPower(POWER);
        }
    }

    public void manualLevel(double manualTarget, double power){
        motorGlisiera1.setTargetPosition((int) manualTarget);
        motorGlisiera2.setTargetPosition((int) manualTarget);
        motorGlisiera1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorGlisiera2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        if(motorGlisiera1.getCurrentPosition() < manualTarget )
        {
            motorGlisiera1.setPower(power);
        }
        else{
            motorGlisiera1.setPower(-power);
        }
        if(motorGlisiera2.getCurrentPosition() < manualTarget )
        {
            motorGlisiera2.setPower(-power);
        }
        else{
            motorGlisiera2.setPower(power);
        }
    }

    public void resetMotors(){
        motorGlisiera1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorGlisiera2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void coboaraCuva(){
        double pos = 0.08;
        servoStanga.setPosition(pos);
        servoDreapta.setPosition(pos);
    }

    public void ridicaCuva(){
        double pos = 0.85;
        servoStanga.setPosition(pos);
        servoDreapta.setPosition(pos);
    }

    public void stangaCuva(){
        servoRotCuva.setPosition(0.15);
    }

    public void dreaptaCuva(){
        servoRotCuva.setPosition(0.85);
    }

    public void susCuva(){
        servoRotCuva.setPosition(0.5);
    }

    public void setCuva(double pos){
        servoRotCuva.setPosition(pos);
    }
}