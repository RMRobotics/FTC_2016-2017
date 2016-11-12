package org.firstinspires.ftc.rmrobotics.util.config;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.rmrobotics.hardware.motor;
import org.firstinspires.ftc.rmrobotics.util.MotorType;

import java.util.Arrays;

public class TheRMite extends Robot{
    private motor FL;
    private motor FR;
    private motor BL;
    private motor BR;

    public TheRMite(final HardwareMap h) {
        super(h);
    }

    @Override
    protected void config() {
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
}