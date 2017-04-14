package org.firstinspires.ftc.rmrobotics.util.config;

import com.kauailabs.navx.ftc.AHRS;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.rmrobotics.hardware.i2csensor;
import org.firstinspires.ftc.rmrobotics.hardware.crservo;
import org.firstinspires.ftc.rmrobotics.hardware.motor;
import org.firstinspires.ftc.rmrobotics.hardware.servo;
import org.firstinspires.ftc.rmrobotics.util.enums.Op;

import java.util.ArrayList;

public abstract class Robot {

    protected HardwareMap hMap;
    protected ArrayList<motor> motors = new ArrayList<>();
    protected ArrayList<servo> servos = new ArrayList<>();
    protected ArrayList<crservo> crservos = new ArrayList<>();
    protected ArrayList<i2csensor> sensors = new ArrayList<>();
    protected AHRS navx;
    protected Op opMode;
    protected DcMotor.RunMode motorMode;

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

    public ArrayList sensors() {
        return sensors;
    }
}
