package org.firstinspires.ftc.rmrobotics.opmodes.feRMilab;

import com.kauailabs.navx.ftc.AHRS;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchImpl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Created by Simon on 1/6/16.
 */
// RED TEAMMMMMMMMMMMMMMMMMMMEMEMMMMMMMMMMMMM


@Autonomous(name = "sensors9")
@Disabled
public class sensorAutoLinear5 extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();

    private DcMotor FL;
    private DcMotor FR;
    private DcMotor BL;
    private DcMotor BR;

    private Servo swingArm;

    private AHRS navx;

    private I2cDevice colorBack;
    private I2cDeviceSynch colorBackReader;
    private I2cDevice colorCenter;
    private I2cDeviceSynch colorCenterReader;
    private I2cDevice colorRight;
    private I2cDeviceSynch colorRightReader;
    private I2cDevice colorLeft;
    private I2cDeviceSynch colorLeftReader;

    I2cDevice range;
    I2cDeviceSynch rangeReader;

    @Override
    public void runOpMode() throws InterruptedException {
        FL = hardwareMap.dcMotor.get("FL");
        FR = hardwareMap.dcMotor.get("FR");
        BL = hardwareMap.dcMotor.get("BL");
        BR = hardwareMap.dcMotor.get("BR");
        swingArm = hardwareMap.servo.get("swingArm");
        FR.setDirection(DcMotor.Direction.REVERSE);
        BR.setDirection(DcMotor.Direction.REVERSE);
        setZeroMode(DcMotor.ZeroPowerBehavior.BRAKE);
        setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setMode(DcMotor.RunMode.RUN_TO_POSITION);

        navx = AHRS.getInstance(hardwareMap.deviceInterfaceModule.get("dim"), 0, AHRS.DeviceDataType.kProcessedData, (byte) 50);

        while (navx.isCalibrating()) {
            telemetry.addData("Status", !navx.isCalibrating());
            telemetry.update();
        }

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        colorBack = hardwareMap.i2cDevice.get("colorBack");
        colorBackReader = new I2cDeviceSynchImpl(colorBack, I2cAddr.create8bit(0x50), false);
        colorBackReader.engage();
        colorBackReader.write8(3,0);

        colorCenter = hardwareMap.i2cDevice.get("colorCenter");
        colorCenterReader = new I2cDeviceSynchImpl(colorCenter, I2cAddr.create8bit(0x52), false);
        colorCenterReader.engage();
        colorCenterReader.write8(3,0);

        colorRight = hardwareMap.i2cDevice.get("colorRight");
        colorRightReader = new I2cDeviceSynchImpl(colorRight, I2cAddr.create8bit(0x70), false);
        colorRightReader.engage();
        colorRightReader.write8(3,1);

        colorLeft = hardwareMap.i2cDevice.get("colorLeft");
        colorLeftReader = new I2cDeviceSynchImpl(colorLeft, I2cAddr.create8bit(0x72), false);
        colorLeftReader.engage();
        colorLeftReader.write8(3,1);

        range = hardwareMap.i2cDevice.get("range");
        rangeReader = new I2cDeviceSynchImpl(range, I2cAddr.create8bit(0x60), false);
        rangeReader.engage();

        double startTime;

        waitForStart();
        runtime.reset(); // reset runtime counter
        navx.zeroYaw(); // reset navx yaw value

        // manuever into shooting position
        setEnc(560, 560, 560, 560);
        setDrive(0.2, 0.2, 0.2, 0.2);
        sleep(1000);

        // TODO: insert shooter code

        // turn towards first beacon TODO: change to rotation with all wheels
        setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        while (Math.abs(navx.getYaw() + 45) > 2) {
            if (Math.abs(navx.getYaw()) < 35) {
                setDrive(0, 0.2, 0, 0.2);
            } else {
                setDrive(0, 0.07, 0, 0.07);
            }
            addTelemetry();
        }

        // drive forward until center color sensor detects line
        while (colorCenterReader.read(0x08, 1)[0] < 25) {
            if (FR.getCurrentPosition() < 4200) {
                setDrive(0.3, 0.3, 0.3, 0.3); // test fastest speed optimal
            } else {
                setDrive(0.06, 0.06, 0.06, 0.06);
            }
            addTelemetry();
        }

        setDrive(0, 0, 0, 0);

        // turn until back color sensor also detects the line
        while (colorBackReader.read(0x08, 1)[0] < 25) {
            if (Math.abs(navx.getYaw() - 90) > 15) {
                setDrive(0.2, -0.2, 0.2, -0.2);
            } else {
                setDrive(0.06, -0.06, 0.06, -0.06);
            }
        }

        setDrive(0, 0, 0, 0);

        // drive forward until close enough to beacon
        while (rangeReader.read(0x04, 2)[0] > 11) {
            setDrive(-0.1, -0.1, -0.1, -0.1);
            addTelemetry();
        }

        setDrive(0, 0, 0, 0);

        // give beacon pusher enough time to detect color of beacon

//        while (runtime.milliseconds() - startTime < 2500) {
//            addTelemetry();
//        }

        // move beacon pusher arm to appropriate location
        if (colorLeftReader.read(0x04, 1)[0] == 10) {// && colorRightCache[0] == 3){
            //left is red, right is blue
            swingArm.setPosition(.72);
        }
        else if (colorLeftReader.read(0x04, 1)[0] == 3) {// && colorRightCache[0] == 10){
            swingArm.setPosition(.15);
        }
        addTelemetry();

        sleep(500);

        // drive forward to hit beacon
        while (rangeReader.read(0x04, 2)[1] < 12) {
            if (colorLeftReader.read(0x04,1)[0] == 10){
                setDrive(0,0,0,0);
            }else {
                setDrive(-0.1, -0.1, -0.1, -0.1);
            }
            addTelemetry();
        }

        // back away from beacon
        while (rangeReader.read(0x04, 2)[0] < 20) {
            setDrive(0.1, 0.1, 0.1, 0.1);
            addTelemetry();
        }


        setDrive(0, 0, 0, 0);

        // turn towards second line
        while (Math.abs(navx.getYaw() + 5) > 5) {
            if (Math.abs(navx.getYaw()) > 25) {
                setDrive(0.2, -0.2, 0.2, -0.2);
            } else {
                setDrive(0.07, -0.07, 0.07, -0.07);
            }
            addTelemetry();
        }

        // drive forward slightly to move center color sensor off the first line
        double start = runtime.milliseconds();
        while (runtime.milliseconds() - start < 1000) {
            setDrive(-0.2, -0.2, -0.2, -0.2);
            addTelemetry();
        }

        // drive forward until center color sensor detects second line
        while (colorCenterReader.read(0x08, 1)[0] < 15) {
            if (FR.getCurrentPosition() > -5000) {
                setDrive(-0.3, -0.3, -0.3, -0.3); // test top speed
            } else {
                setDrive(-0.06, -0.06, -0.06, -0.06);
            }
            addTelemetry();
        }

        // turn until back color sensor also detects the line
        while (colorBackReader.read(0x08, 1)[0] < 25) {
            if (Math.abs(navx.getYaw() - 85) > 5) { //test encoder value
                setDrive(-0.1, 0.1, -0.1, 0.1);
            } else {
                setDrive(-0.06, 0.06, -0.06, 0.06);
            }
        }

        // drive forward until close enough to beacon
        while (rangeReader.read(0x04, 2)[0] > 11) {
            setDrive(-0.1, -0.1, -0.1, -0.1);
            addTelemetry();
        }

        setDrive(0, 0, 0, 0);

        // give beacon pusher enough time to detect color of beacon
//        startTime = runtime.milliseconds();
//        while (runtime.milliseconds() - startTime < 2500) {
//            addTelemetry();
//        }

        // move beacon pusher arm to appropriate location
        if (colorLeftReader.read(0x04, 1)[0] == 10) {// && colorRightCache[0] == 3){
            //left is red, right is blue
            swingArm.setPosition(.72);
        }
        else if (colorLeftReader.read(0x04, 1)[0] == 3) {// && colorRightCache[0] == 10){
            swingArm.setPosition(.15);
        }
        addTelemetry();
        sleep(500);

        // drive forward to hit beacon
        while (rangeReader.read(0x04, 2)[1] < 12) {
            if (colorLeftReader.read(0x04,1)[0] == 10){
                setDrive(0,0,0,0);
            }else {
                setDrive(-0.1, -0.1, -0.1, -0.1);
            }
            addTelemetry();
        }

        // back away from beacon
        while (rangeReader.read(0x04, 2)[0] < 20) {
            setDrive(0.1, 0.1, 0.1, 0.1);
            addTelemetry();
        }

        setDrive(0, 0, 0, 0);

        while (Math.abs(navx.getYaw() - 135) > 3){
            if (Math.abs(navx.getYaw()) < 115) {
                setDrive(0.4, 0, 0.4, 0);
            } else {
                setDrive(0.1, 0, 0.1, 0);
            }
            addTelemetry();
        }

        setDrive(0, 0, 0, 0);

        double currentFLencoder = FR.getCurrentPosition();
        while (FR.getCurrentPosition() - currentFLencoder < -600){
            setDrive(.4, .4, .4, .4);
        }

        setDrive(0, 0, 0, 0);

        stop();
    }

    private void addTelemetry() {
        telemetry.addData("1 Time", runtime.seconds());
        telemetry.addData("2 Yaw", navx.getYaw());
        telemetry.addData("3 ColorBack", colorBackReader.read(0x08, 1)[0] & 0xFF);
        telemetry.addData("4 ColorCenter", colorCenterReader.read(0x08, 1)[0] & 0xFF);
        telemetry.addData("5 ColorLeft", colorLeftReader.read(0x04, 1)[0] & 0xFF);
        telemetry.addData("6 ColorRight", colorRightReader.read(0x04, 1)[0] & 0xFF);
        telemetry.addData("7 Range", rangeReader.read(0x04, 2)[0] + " " + rangeReader.read(0x04, 2)[1]);
        telemetry.addData("8 Motor", FL.getPower() + " " + FR.getPower() + " " + BL.getPower() + " " + BR.getPower());
        telemetry.addData("9 Encoder", FL.getCurrentPosition() + " " + FR.getCurrentPosition() + " " + BL.getCurrentPosition() + " " + BR.getCurrentPosition());
        telemetry.update();
    }

    private void setMode(DcMotor.RunMode r) {
        FL.setMode(r);
        FR.setMode(r);
        BL.setMode(r);
        BR.setMode(r);
    }

    private void setZeroMode(DcMotor.ZeroPowerBehavior z) {
        FL.setZeroPowerBehavior(z);
        FR.setZeroPowerBehavior(z);
        BL.setZeroPowerBehavior(z);
        BR.setZeroPowerBehavior(z);
    }

    private void setDrive(double p1, double p2, double p3, double p4) {
        FL.setPower(p1);
        FR.setPower(p2);
        BL.setPower(p3);
        BR.setPower(p4);
    }

    private void setEnc(int p1, int p2, int p3, int p4) {
        FL.setTargetPosition(FL.getCurrentPosition() + p1);
        FR.setTargetPosition(FR.getCurrentPosition() + p2);
        BL.setTargetPosition(BL.getCurrentPosition() + p3);
        BR.setTargetPosition(BR.getCurrentPosition() + p4);
    }

}