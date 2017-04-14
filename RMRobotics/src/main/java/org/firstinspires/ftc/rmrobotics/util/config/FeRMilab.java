package org.firstinspires.ftc.rmrobotics.util.config;

import com.kauailabs.navx.ftc.AHRS;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.rmrobotics.hardware.i2csensor;
import org.firstinspires.ftc.rmrobotics.hardware.crservo;
import org.firstinspires.ftc.rmrobotics.hardware.motor;
import org.firstinspires.ftc.rmrobotics.hardware.servo;
import org.firstinspires.ftc.rmrobotics.util.enums.MotorType;
import org.firstinspires.ftc.rmrobotics.util.enums.Op;
import org.firstinspires.ftc.rmrobotics.util.enums.Sensors;

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
    private motor lift;
    private motor belt;

    private servo index;
    private servo liftHold;

    private crservo beaconL;
    private crservo beaconR;
    //private servo fork;

    protected VoltageSensor flyMC;

    protected i2csensor colorCenter;
    protected i2csensor colorRight;
    protected i2csensor colorLeft;
    protected i2csensor range;

    public FeRMilab(final HardwareMap h, final Op o) {
        super(h);
        opMode = o;
    }

    public FeRMilab(final HardwareMap h, final DcMotor.RunMode runMode) {
        super(h);
        motorMode = runMode;
    }

    @Override
    protected void config() {
        if (opMode != null) {
            if (opMode == Op.TELEOP) {
                motorMode = DcMotor.RunMode.RUN_USING_ENCODER;
            } else {
                motorMode = DcMotor.RunMode.RUN_TO_POSITION;
            }
        }

        navx = AHRS.getInstance(hMap.deviceInterfaceModule.get("dim"), 0, AHRS.DeviceDataType.kProcessedData, (byte) 50);

        FL = new motor(hMap.dcMotor.get("FL"), DcMotorSimple.Direction.REVERSE, motorMode, DcMotor.ZeroPowerBehavior.FLOAT, MotorType.NVRST40);
        FR = new motor(hMap.dcMotor.get("FR"), DcMotorSimple.Direction.FORWARD, motorMode, DcMotor.ZeroPowerBehavior.FLOAT, MotorType.NVRST40);
        BL = new motor(hMap.dcMotor.get("BL"), DcMotorSimple.Direction.REVERSE, motorMode, DcMotor.ZeroPowerBehavior.FLOAT, MotorType.NVRST40);
        BR = new motor(hMap.dcMotor.get("BR"), DcMotorSimple.Direction.FORWARD, motorMode, DcMotor.ZeroPowerBehavior.FLOAT, MotorType.NVRST40);
        flyL = new motor(hMap.dcMotor.get("flyL"), DcMotorSimple.Direction.REVERSE, DcMotor.RunMode.RUN_USING_ENCODER, DcMotor.ZeroPowerBehavior.FLOAT, MotorType.NVRST);
        flyR = new motor(hMap.dcMotor.get("flyR"), DcMotorSimple.Direction.REVERSE, DcMotor.RunMode.RUN_USING_ENCODER, DcMotor.ZeroPowerBehavior.FLOAT, MotorType.NVRST);
        lift = new motor(hMap.dcMotor.get("lift"), DcMotorSimple.Direction.REVERSE, DcMotor.RunMode.RUN_USING_ENCODER, DcMotor.ZeroPowerBehavior.BRAKE, MotorType.NVRST40);
        belt = new motor(hMap.dcMotor.get("belt"), DcMotorSimple.Direction.REVERSE, DcMotor.RunMode.RUN_WITHOUT_ENCODER, DcMotor.ZeroPowerBehavior.BRAKE, MotorType.NVRST40);
        motors.addAll(Arrays.asList(FL, FR, BL, BR, flyL, flyR, lift, belt));

        index = new servo(hMap.servo.get("indexer"), Servo.Direction.FORWARD, 0.12, 0.5, 0.12);
        liftHold = new servo(hMap.servo.get("liftHold"), Servo.Direction.FORWARD, 0, 1, 0.4);
        servos.addAll(Arrays.asList(index, liftHold));

        beaconL = new crservo(hMap.crservo.get("beaconL"), CRServo.Direction.FORWARD);
        beaconR = new crservo(hMap.crservo.get("beaconR"), CRServo.Direction.REVERSE);
        crservos.addAll(Arrays.asList(beaconL, beaconR));

        colorCenter = new i2csensor(hMap.i2cDevice.get("colorCenter"), (byte) 0x52, Sensors.COLORON);
        colorRight = new i2csensor(hMap.i2cDevice.get("colorRight"), (byte) 0x70, Sensors.COLOROFF);
        colorLeft = new i2csensor(hMap.i2cDevice.get("colorLeft"), (byte) 0x72, Sensors.COLOROFF);
        range = new i2csensor(hMap.i2cDevice.get("range"), (byte) 0x60, Sensors.RANGE);
        sensors.addAll(Arrays.asList(colorCenter, colorRight, colorLeft, range));
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
    public motor lift() {
        return lift;
    }
    public motor belt() {
        return belt;
    }

    public servo index() {
        return index;
    }
    public servo liftHold() {
        return liftHold;
    }

    public crservo beaconL() {
        return beaconL;
    }
    public crservo beaconR() {
        return beaconR;
    }

    public i2csensor colorCenter() {
        return colorCenter;
    }
    public i2csensor colorRight() {
        return colorRight;
    }
    public i2csensor colorLeft() {
        return colorLeft;
    }
    public i2csensor range() {
        return range;
    }
}
