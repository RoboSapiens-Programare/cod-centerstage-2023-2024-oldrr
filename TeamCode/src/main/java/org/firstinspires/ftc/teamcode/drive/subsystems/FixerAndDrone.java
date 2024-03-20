package org.firstinspires.ftc.teamcode.drive.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class FixerAndDrone {
    private Servo servoAvion;
//    private Servo mozaicFixer;

    public FixerAndDrone(HardwareMap hardwareMap){
        servoAvion = hardwareMap.servo.get("servoAvion");
//        mozaicFixer = hardwareMap.servo.get("mozaicFixer");

        servoAvion.setDirection(Servo.Direction.FORWARD);
//        mozaicFixer.setDirection(Servo.Direction.FORWARD);
    }

    public void activateDrone(){
        servoAvion.setPosition(1);
    }

    public void releaseDrone(){
        servoAvion.setPosition(0);
    }

//    public void setFixer(double pos){
//        mozaicFixer.setPosition(pos);
//    }
}
