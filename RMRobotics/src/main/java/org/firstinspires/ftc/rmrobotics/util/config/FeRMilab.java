package org.firstinspires.ftc.rmrobotics.util.config;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.rmrobotics.hardware.crservo;
import org.firstinspires.ftc.rmrobotics.hardware.motor;
import org.firstinspires.ftc.rmrobotics.hardware.servo;
import org.firstinspires.ftc.rmrobotics.util.MotorType;
import org.firstinspires.ftc.rmrobotics.util.Robot;

import java.util.Arrays;

/**
 * Created by Simon on 11/30/16.
 */

public class FeRMilab extends Robot {
    private motor FL;
    private motor FR;
    private motor BL;
    private motor BR;
    private motor flyL;
    private motor flyR;
    private motor liftL;
    private motor liftR;

    private servo beaconL;
    private servo beaconR;
    //private servo fork;

    private servo harvester;
    private servo belt;

    public FeRMilab(final HardwareMap h, final boolean t) {
        super(h);
        opMode = t;
    }

    @Override
    protected void config() {
        if (!opMode) {
            motorMode = DcMotor.RunMode.RUN_TO_POSITION;
        } else {
            motorMode = DcMotor.RunMode.RUN_USING_ENCODER;
        }

        FL = new motor(hMap.dcMotor.get("FL"), DcMotorSimple.Direction.FORWARD, motorMode, DcMotor.ZeroPowerBehavior.FLOAT, MotorType.NVRST40);
        FR = new motor(hMap.dcMotor.get("FR"), DcMotorSimple.Direction.REVERSE, motorMode, DcMotor.ZeroPowerBehavior.FLOAT, MotorType.NVRST40);
        BL = new motor(hMap.dcMotor.get("BL"), DcMotorSimple.Direction.FORWARD, motorMode, DcMotor.ZeroPowerBehavior.FLOAT, MotorType.NVRST40);
        BR = new motor(hMap.dcMotor.get("BR"), DcMotorSimple.Direction.REVERSE, motorMode, DcMotor.ZeroPowerBehavior.FLOAT, MotorType.NVRST40);
        flyL = new motor(hMap.dcMotor.get("flyL"), DcMotorSimple.Direction.FORWARD, DcMotor.RunMode.RUN_USING_ENCODER, DcMotor.ZeroPowerBehavior.FLOAT, MotorType.NVRST40);
        flyR = new motor(hMap.dcMotor.get("flyR"), DcMotorSimple.Direction.FORWARD, DcMotor.RunMode.RUN_USING_ENCODER, DcMotor.ZeroPowerBehavior.FLOAT, MotorType.NVRST40);
        liftL = new motor(hMap.dcMotor.get("liftL"), DcMotorSimple.Direction.FORWARD, DcMotor.RunMode.RUN_USING_ENCODER, DcMotor.ZeroPowerBehavior.BRAKE, MotorType.NVRST40);
        liftR = new motor(hMap.dcMotor.get("liftR"), DcMotorSimple.Direction.REVERSE, DcMotor.RunMode.RUN_USING_ENCODER, DcMotor.ZeroPowerBehavior.BRAKE, MotorType.NVRST40);
        motors.addAll(Arrays.asList(FL, FR, BL, BR, flyL, flyR, liftL, liftR));

        beaconL = new servo(hMap.servo.get("beaconL"), Servo.Direction.FORWARD, 0, 1, 0);
        beaconR = new servo(hMap.servo.get("beaconR"), Servo.Direction.REVERSE, 0, 1, 0);
        //fork = new servo(hMap.servo.get("fork"), Servo.Direction.FORWARD, 0, 1, 0);
        harvester = new servo(hMap.servo.get("h"), Servo.Direction.FORWARD, 0, 1, 0);
        belt = new servo(hMap.servo.get("b"), Servo.Direction.FORWARD, 0, 1, 0);
        servos.addAll(Arrays.asList(beaconL, beaconR, harvester, belt));
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

    public motor flyL() {
        return flyL;
    }

    public motor flyR() {
        return flyR;
    }

    public motor liftL() {
        return liftL;
    }

    public motor liftR() {
        return liftR;
    }

    public servo beaconL() {
        return beaconL;
    }

    public servo beaconR() {
        return beaconR;
    }

    /*public servo fork() {
        return fork;
    }*/

    public servo harvester() {
        return harvester;
    }

    public servo belt() {
        return belt;
    }
}
