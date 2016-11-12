package org.firstinspires.ftc.rmrobotics.util.config;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.rmrobotics.hardware.crservo;
import org.firstinspires.ftc.rmrobotics.hardware.motor;
import org.firstinspires.ftc.rmrobotics.hardware.servo;

import java.util.ArrayList;

public abstract class Robot {

    protected HardwareMap hMap;
    protected ArrayList<motor> motors;
    protected ArrayList<servo> servos;
    protected ArrayList<crservo> crservos;

    public Robot(HardwareMap h){
        hMap = h;
        this.config();
    }

    protected abstract void config();

    public ArrayList motors() {
        return motors;
    }

    public ArrayList servos() {
        return servos;
    }

    public ArrayList crservos() {
        return crservos;
    }
}
