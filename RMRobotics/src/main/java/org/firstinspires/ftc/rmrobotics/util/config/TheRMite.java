package org.firstinspires.ftc.rmrobotics.util.config;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.rmrobotics.hardware.crservo;
import org.firstinspires.ftc.rmrobotics.hardware.motor;
import org.firstinspires.ftc.rmrobotics.hardware.servo;
import org.firstinspires.ftc.rmrobotics.util.MotorType;

import java.util.ArrayList;
import java.util.Arrays;

/**
 * Created by Simon on 11/11/2016.
 */

public class TheRMite implements Robot{
    private HardwareMap hMap;
    private ArrayList<motor> motors;
    private ArrayList<servo> servos;
    private ArrayList<crservo> crservos;

    private motor FL;
    private motor FR;
    private motor BL;
    private motor BR;

    public  TheRMite(final HardwareMap h) {
        hMap = h;
        this.config();
    }

    private void config() {
        FL = new motor(hMap.dcMotor.get("FL"), DcMotorSimple.Direction.FORWARD, DcMotor.RunMode.RUN_USING_ENCODER, DcMotor.ZeroPowerBehavior.FLOAT, MotorType.NVRST40);
        FR = new motor(hMap.dcMotor.get("FR"), DcMotorSimple.Direction.REVERSE, DcMotor.RunMode.RUN_USING_ENCODER, DcMotor.ZeroPowerBehavior.FLOAT, MotorType.NVRST40);
        BL = new motor(hMap.dcMotor.get("BL"), DcMotorSimple.Direction.FORWARD, DcMotor.RunMode.RUN_USING_ENCODER, DcMotor.ZeroPowerBehavior.FLOAT, MotorType.NVRST40);
        BR = new motor(hMap.dcMotor.get("BR"), DcMotorSimple.Direction.REVERSE, DcMotor.RunMode.RUN_USING_ENCODER, DcMotor.ZeroPowerBehavior.FLOAT, MotorType.NVRST40);
        motors.addAll(Arrays.asList(FL, FR, BL, BR));
    }

    public motor FL() {
        return FL;
    }

    public motor FR() {
        return FR;
    }

    public motor BL() {
        return BL;
    }

    public motor BR() {
        return BR;
    }

    @Override
    public ArrayList motors() {
        return motors;
    }

    @Override
    public ArrayList servos() {
        return servos;
    }

    @Override
    public ArrayList crservos() {
        return crservos;
    }
}