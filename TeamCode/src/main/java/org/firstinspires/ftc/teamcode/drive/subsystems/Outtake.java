package org.firstinspires.ftc.teamcode.drive.subsystems;

import static java.lang.Math.abs;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Outtake {
    private static final double POWER = 1;

    public DcMotor motorGlisiera;
    public Servo servoStanga, servoDreapta; /** Cand te uiti din spatele robotului **/
    public Servo servoCuvaStanga, servoCuvaDreapta;
    public double manualTarget = 0;

    public DistanceSensor senzorDistanta1, senzorDistanta2;



    public Outtake(HardwareMap hardwareMap){
        motorGlisiera = hardwareMap.dcMotor.get("motorGlisiera");

        servoStanga = hardwareMap.servo.get("servoStanga");
        servoDreapta = hardwareMap.servo.get("servoDreapta");

        servoCuvaStanga = hardwareMap.servo.get("servoCuvaStanga");
        servoCuvaDreapta = hardwareMap.servo.get("servoCuvaDreapta");


        senzorDistanta1 = hardwareMap.get(DistanceSensor.class, "senzorDistanta1");
        senzorDistanta2 = hardwareMap.get(DistanceSensor.class, "senzorDistanta2");

        //Motor initialization
        motorGlisiera.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorGlisiera.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorGlisiera.setDirection(DcMotorSimple.Direction.FORWARD);

        servoStanga.setDirection(Servo.Direction.FORWARD);
        servoDreapta.setDirection(Servo.Direction.REVERSE);
        servoCuvaStanga.setDirection(Servo.Direction.REVERSE);
        servoCuvaDreapta.setDirection(Servo.Direction.FORWARD);
    }


    public double calculateThrottle(float x) {
        int sign = -1;
        if (x > 0) sign = 1;
        return sign * Math.pow(100 * (abs(x) / 100), 2);
    }


    public void manualLevel(double manualTarget) {
        motorGlisiera.setTargetPosition((int) manualTarget);
        motorGlisiera.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        if(motorGlisiera.getCurrentPosition() < manualTarget )
        {
            motorGlisiera.setPower(POWER);
        }
        else{
            motorGlisiera.setPower(-POWER);
        }
    }

    public void ridicaCuva(){
        servoStanga.setPosition(0.3);
        servoDreapta.setPosition(0.3);
    }

    public void coboaraCuva(){
        servoStanga.setPosition(0.03);
        servoDreapta.setPosition(0.03);
    }

    public void setCuva(double pos){
        servoStanga.setPosition(pos);
        servoDreapta.setPosition(pos);
    }

    public void deschideCuva(){
        servoCuvaStanga.setPosition(0.2);
        servoCuvaDreapta.setPosition(0.2);
    }
    public void inchideCuva(){
        servoCuvaStanga.setPosition(0);
        servoCuvaDreapta.setPosition(0);
    }
    public void deschideStanga(){
        servoCuvaStanga.setPosition(0.2);
    }
    public void deschideDreapta(){
        servoCuvaDreapta.setPosition(0.2);
    }
    public void inchideStanga(){
        servoCuvaStanga.setPosition(0);
    }
    public void inchideDreapta(){
        servoCuvaDreapta.setPosition(0);
    }



    public boolean pixelStanga(){
        if(senzorDistanta1.getDistance(DistanceUnit.CM) <= 3.8){
            return true;
        }
        else return false;
    }
    public boolean pixelDreapta(){
        if(senzorDistanta2.getDistance(DistanceUnit.CM) <= 3.8){
            return true;
        }
        else return false;
    }

}
