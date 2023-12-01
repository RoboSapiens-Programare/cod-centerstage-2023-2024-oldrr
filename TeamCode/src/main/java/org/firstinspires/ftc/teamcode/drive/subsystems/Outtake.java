package org.firstinspires.ftc.teamcode.drive.subsystems;

import static java.lang.Math.abs;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Outtake {
    private static final double POWER = 1;

    public DcMotor motorGlisiera;
    public Servo servoStanga, servoDreapta; /** Cand te uiti din spatele robotului **/
    public Servo servoCuva;
    public double manualTarget = 0;




    public Outtake(HardwareMap hardwareMap){
        motorGlisiera = hardwareMap.dcMotor.get("motorGlisiera");

        servoStanga = hardwareMap.servo.get("servoStanga");
        servoDreapta = hardwareMap.servo.get("servoDreapta");
        servoCuva = hardwareMap.servo.get("servoCuva");

        //Motor initialization
        motorGlisiera.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorGlisiera.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorGlisiera.setDirection(DcMotorSimple.Direction.FORWARD);

        servoStanga.setDirection(Servo.Direction.FORWARD);
        servoDreapta.setDirection(Servo.Direction.REVERSE);
        servoCuva.setDirection(Servo.Direction.FORWARD);
    }


    public double calculateThrottle(float x) {
        int sign = -1;
        if (x > 0) sign = 1;
        return sign * Math.pow(100 * (abs(x) / 100), 2);
    }


    public void manualLevel(double manualTarget) {
        motorGlisiera.setTargetPosition((int) manualTarget);
        motorGlisiera.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        if(motorGlisiera.getCurrentPosition() > manualTarget )
        {
            motorGlisiera.setPower(POWER);
        }
        else{
            motorGlisiera.setPower(-POWER);
        }
    }

    public void ridicaCuva(){
        servoStanga.setPosition(0.8);
        servoDreapta.setPosition(0.8);
    }

    public void coboaraCuva(){
        servoStanga.setPosition(0.05);
        servoDreapta.setPosition(0.05);
    }

    public void deschideCuva(){
        servoCuva.setPosition(0);
    }

    public void inchideCuva(){
        servoCuva.setPosition(0.35);
    }
}
