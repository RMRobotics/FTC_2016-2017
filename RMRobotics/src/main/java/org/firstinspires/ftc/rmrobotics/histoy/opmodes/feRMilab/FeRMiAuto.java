package org.firstinspires.ftc.rmrobotics.histoy.opmodes.feRMilab;


import com.kauailabs.navx.ftc.AHRS;
import com.kauailabs.navx.ftc.navXPIDController;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchImpl;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by Simon on 12/6/16.
 */

@Autonomous(name = "feRMi - AUTONOMOUS", group = "feRMi")
@Disabled
public class FeRMiAuto extends LinearOpMode{

    private DcMotor FL;
    private DcMotor FR;
    private DcMotor BL;
    private DcMotor BR;
    private DcMotor flyL;
    private DcMotor flyR;
    private DcMotor belt;

    private Servo beaconL;
    private Servo beaconR;
    private Servo harvester;
    private Servo index;

    private final int NAVX_DIM_I2C_PORT = 0;
    private AHRS navx;
    private navXPIDController yawPIDController;
    private final byte NAVX_UPDATE_RATE_HZ = 50;

    private double TARGET_ANGLE_DEGREES = 0;
    private final double TOLERANCE_DEGREES = 1.5;
    private final double MIN_MOTOR_OUTPUT_VALUE = -0.2;
    private final double MAX_MOTOR_OUTPUT_VALUE = 0.2;
    private final double YAW_PID_P = 0.005;
    private final double YAW_PID_I = 0.0;
    private final double YAW_PID_D = 0.0;
    navXPIDController.PIDResult yawPIDResult;

    byte[] linecache;
    I2cDevice line;
    I2cDeviceSynch linereader;

    @Override
    public void runOpMode() throws InterruptedException {
        FL = hardwareMap.dcMotor.get("FL");
        FL.setDirection(DcMotorSimple.Direction.REVERSE);
        FR = hardwareMap.dcMotor.get("FR");
        BL = hardwareMap.dcMotor.get("BL");
        BL.setDirection(DcMotorSimple.Direction.REVERSE);
        BR = hardwareMap.dcMotor.get("BR");
        flyL = hardwareMap.dcMotor.get("flyL");
        flyL.setDirection(DcMotorSimple.Direction.REVERSE);
        flyR = hardwareMap.dcMotor.get("flyR");
        flyR.setDirection(DcMotorSimple.Direction.REVERSE);
        belt = hardwareMap.dcMotor.get("b");
        beaconL = hardwareMap.servo.get("beaconL");
        beaconR = hardwareMap.servo.get("beaconR");
        beaconR.setDirection(Servo.Direction.REVERSE);
        harvester = hardwareMap.servo.get("h");
        index = hardwareMap.servo.get("indexer");

        navx = AHRS.getInstance(hardwareMap.deviceInterfaceModule.get("dim"), NAVX_DIM_I2C_PORT, AHRS.DeviceDataType.kProcessedData, NAVX_UPDATE_RATE_HZ);
        yawPIDController = new navXPIDController(navx, navXPIDController.navXTimestampedDataSource.YAW);
        yawPIDController.setSetpoint(TARGET_ANGLE_DEGREES);
        yawPIDController.setContinuous(true);
        yawPIDController.setOutputRange(MIN_MOTOR_OUTPUT_VALUE, MAX_MOTOR_OUTPUT_VALUE);
        yawPIDController.setTolerance(navXPIDController.ToleranceType.ABSOLUTE, TOLERANCE_DEGREES);
        yawPIDController.setPID(YAW_PID_P, YAW_PID_I, YAW_PID_D);
        yawPIDController.enable(true);

        line = hardwareMap.i2cDevice.get("cL");
        linereader = new I2cDeviceSynchImpl(line, I2cAddr.create8bit(0x50), false);
        linereader.engage();
        linereader.write8(3, 0);

        waitForStart();
        navx.zeroYaw();
        yawPIDResult = new navXPIDController.PIDResult();

        setEncoder(1120, 0.2);
        sleep(2000);

        flyL.setPower(1.0);
        flyR.setPower(1.0);
        belt.setPower(1.0);
        sleep(500);

        index.setPosition(1.0);
        sleep(7500);

        index.setPosition(0);
        flyL.setPower(0);
        flyR.setPower(0);
        belt.setPower(0);
        while (!yawPIDController.isOnTarget()) {
            double output = yawPIDResult.getOutput();
            setDrive(output, -output);
        }
        while (linereader.read(0x04, 1)[0] != 16) {
            double output = yawPIDResult.getOutput();
            setDrive(0.3 + output, 0.3 - output);
        }
    }

    private void setDrive(double p) {
        FL.setPower(p);
        FR.setPower(p);
        BL.setPower(p);
        BR.setPower(p);
    }

    private void setDrive(double p1, double p2) {
        FL.setPower(p1);
        FR.setPower(p2);
        BL.setPower(p1);
        BR.setPower(p2);
    }

    private void setDrive(double p1, double p2, double p3, double p4) {
        FL.setPower(p1);
        FR.setPower(p2);
        BL.setPower(p3);
        BR.setPower(p4);
    }

    private void setEncoder(int e, double p) {
        FL.setTargetPosition(FL.getCurrentPosition() + e);
        FL.setPower(p);
        FR.setTargetPosition(FR.getCurrentPosition() + e);
        FR.setPower(p);
        BL.setTargetPosition(BL.getCurrentPosition() + e);
        BL.setPower(p);
        BR.setTargetPosition(BR.getCurrentPosition() + e);
        BR.setPower(p);
    }

    private void setEncoder(int e1, int e2, double p1, double p2) {
        FL.setTargetPosition(FL.getCurrentPosition() + e1);
        FL.setPower(p1);
        FR.setTargetPosition(FR.getCurrentPosition() + e2);
        FR.setPower(p2);
        BL.setTargetPosition(BL.getCurrentPosition() + e1);
        BL.setPower(p1);
        BR.setTargetPosition(BR.getCurrentPosition() + e2);
        BR.setPower(p2);
    }

    private void setEncoder(int e1, int e2, int e3, int e4, double p1, double p2, double p3, double p4) {
        FL.setTargetPosition(FL.getCurrentPosition() + e1);
        FL.setPower(p1);
        FR.setTargetPosition(FR.getCurrentPosition() + e2);
        FR.setPower(p2);
        BL.setTargetPosition(BL.getCurrentPosition() + e3);
        BL.setPower(p3);
        BR.setTargetPosition(BR.getCurrentPosition() + e4);
        BR.setPower(p4);
    }
}
