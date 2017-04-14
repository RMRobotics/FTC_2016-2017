package org.firstinspires.ftc.rmrobotics.histoy.opmodes.feRMilab;

import com.kauailabs.navx.ftc.AHRS;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
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


@Autonomous(name = "Redsensors10")
@Disabled
public class sensorAutoLinear6 extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();

    private DcMotor FL;
    private DcMotor FR;
    private DcMotor BL;
    private DcMotor BR;

    private Servo swingArm;
    private Servo harvester;

    private AHRS navx;

    private I2cDevice colorBack;
    private I2cDeviceSynch colorBackReader;
    private I2cDevice colorCenter;
    private I2cDeviceSynch colorCenterReader;
    private I2cDevice colorRight;
    private I2cDeviceSynch colorRightReader;
    private I2cDevice colorLeft;
    private I2cDeviceSynch colorLeftReader;

    private DeviceInterfaceModule dim;

    I2cDevice range;
    I2cDeviceSynch rangeReader;

    @Override
    public void runOpMode() {
        FL = hardwareMap.dcMotor.get("FL");
        FR = hardwareMap.dcMotor.get("FR");
        BL = hardwareMap.dcMotor.get("BL");
        BR = hardwareMap.dcMotor.get("BR");
        swingArm = hardwareMap.servo.get("swingArm");
        harvester = hardwareMap.servo.get("h");
        FR.setDirection(DcMotor.Direction.REVERSE);
        BR.setDirection(DcMotor.Direction.REVERSE);
        setZeroMode(DcMotor.ZeroPowerBehavior.BRAKE);
        setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setMode(DcMotor.RunMode.RUN_TO_POSITION);

        dim = hardwareMap.deviceInterfaceModule.get("dim");

        navx = AHRS.getInstance(dim, 0, AHRS.DeviceDataType.kProcessedData, (byte) 50);

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

        dim.setLED(1, true);

        navx.zeroYaw();

        waitForStart();
        runtime.reset(); // reset runtime counter
        //navx.zeroYaw(); // reset navx yaw value
        swingArm.setPosition(0.5);
        harvester.setPosition(0.5);

        // manuever into shooting position
//        setEnc(-560, -560, -560, -560);
//        setDrive(0.2, 0.2, 0.2, 0.2);
//        sleep(1000);
//
//        // TODO: insert shooter code
//
//        // turn towards first beacon TODO: change to rotation with all wheels
//        setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        while (Math.abs(navx.getYaw() + 42) > 2 && opModeIsActive()) {
//            int scale;
//            if (navx.getYaw() + 42 > 0) {
//                scale = -1;
//            } else {
//                scale = 1;
//            }
//            if (Math.abs(navx.getYaw()) < 30) {
//                setDrive(scale * 0.25, 0);
//            } else {
//                setDrive(scale * 0.07, 0);
//            }
//        }
//        addTelemetry();

//        setDrive(0);
//        sleep(100);

        double initPos = FL.getCurrentPosition();

        // drive forward until center color sensor detects line
        while (colorCenterReader.read(0x08, 1)[0] < 25 && opModeIsActive()) {
            int scale = -1;
            if (FL.getCurrentPosition() - initPos < -4000) {
                scale = 1;
            } else if (FL.getCurrentPosition() - initPos > -2400) {
                scale = -1;
            }
            if (FL.getCurrentPosition() - initPos > -2500) {
                setDrive(scale * 0.3);
            } else {
                setDrive(scale * 0.07);
            }
        }
        addTelemetry();

        setDrive(0, 0, 0, 0);
        sleep(100);

        while (colorCenterReader.read(0x08, 1)[0] < 25 && opModeIsActive()) {
            setDrive(0.07);
        }

        setDrive(0, 0, 0, 0);
        sleep(100);

        // turn until back color sensor also detects the line
        while (Math.abs(navx.getYaw() + 86) > 5 && opModeIsActive()) {
            int scale;
            if (navx.getYaw() + 86 > 0) {
                scale = -1;
            } else {
                scale = 1;
            }
            if (Math.abs(navx.getYaw() + 86) > 15) {
                setDrive(scale * 0.15, -scale * 0.15);
            } else {
                setDrive(scale * 0.07, -scale * 0.07);
            }
            //addTelemetry();
        }
        addTelemetry();

        setDrive(0, 0, 0, 0);
        sleep(100);

        while (navx.getYaw() > -85 && opModeIsActive()) {
            int scale;
            if (navx.getYaw() + 85 > 0) {
                scale = -1;
            } else {
                scale = 1;
            }
            setDrive(scale * 0.05, -scale * 0.05);
        }
        addTelemetry();

        setDrive(0);
        sleep(100);

        // drive forward until close enough to beacon
        while (rangeReader.read(0x04, 2)[0] > 20 && opModeIsActive()) {
            setDrive(-0.1);
            addTelemetry();
        }
        setDrive(0);

        setDrive(0, 0, 0, 0);
        sleep(100);

////        setDrive(0, 0, 0, 0);
////        sleep(100);
////
////        // give beacon pusher enough time to detect color of beacon
//        double startTime = runtime.milliseconds();
////        while (runtime.milliseconds() - startTime < 500) {
////            addTelemetry();
////        }
//
//        setDrive(0, 0, 0, 0);
//        sleep(100);

        // move beacon pusher arm to appropriate location
        if (Math.abs(colorLeftReader.read(0x04, 1)[0] - 10) <= 1 || Math.abs(colorRightReader.read(0x04, 1)[0] - 3) <= 1) {// && colorRightCache[0] == 3){
            //left is red, right is blue
            swingArm.setPosition(.72);
        }
        else if (Math.abs(colorLeftReader.read(0x04, 1)[0] - 3) <= 1 || Math.abs(colorRightReader.read(0x04, 1)[0] - 10) <= 1) {// && colorRightCache[0] == 10){
            swingArm.setPosition(.15);
        }
        addTelemetry();

        // drive forward to hit beacon
        while (rangeReader.read(0x04, 2)[1] < 12 && opModeIsActive()) {
            setDrive(-0.15, -0.15, -0.15, -0.15);
            addTelemetry();
        }

//        setDrive(0, 0, 0, 0);
//        sleep(100);

        // back away from beacon
        while (rangeReader.read(0x04, 2)[0] < 17 && opModeIsActive()) {
            setDrive(0.2, 0.2, 0.2, 0.2);
            addTelemetry();
        }

//        setDrive(0, 0, 0, 0);
//        sleep(100);

        // turn towards second line
        while (navx.getYaw() < -5 && opModeIsActive()) {
            if (Math.abs(navx.getYaw()) > 25) {
                setDrive(.1, -0.1, .1, -0.1);
            } else {
                setDrive(0, -0.07, 0, -0.07);
            }
            addTelemetry();
        }

//        setDrive(0, 0, 0, 0);
//        sleep(100);

        int curEnc = FL.getCurrentPosition();

        // drive forward slightly to move center color sensor off the first line
        double start = runtime.milliseconds();
        while (runtime.milliseconds() - start < 1000 && opModeIsActive()) {
            setDrive(-0.4);
            addTelemetry();
        }

        // drive forward until center color sensor detects second line
        while (colorCenterReader.read(0x08, 1)[0] < 25 && opModeIsActive()) {
            int scale = -1;
            int relDis = FL.getCurrentPosition() - curEnc;
            if (relDis > -1900) {
                setDrive(-0.4);
            } else {
                if (relDis > -1800) {
                    scale = -1;
                } else if(relDis < -3000) {
                    scale = 1;
                }
                setDrive(scale*0.07);
            }
        }

//        setDrive(0, 0, 0, 0);
//        sleep(100);

        setDrive(0);
        sleep(100);

        while (colorCenterReader.read(0x08, 1)[0] < 25 && opModeIsActive()) {
            setDrive(0.04);
        }
        addTelemetry();

        setDrive(0);
        sleep(100);

        while (Math.abs(navx.getYaw() + 88) > 3 && opModeIsActive()) {
            int scale;
            if (navx.getYaw() + 90 > 0) {
                scale = -1;
            } else {
                scale = 1;
            }
            if (Math.abs(navx.getYaw() + 88) > 15) {
                setDrive(scale * 0.2, -scale * 0.2);
            } else {
                setDrive(scale * 0.07, -scale * 0.07);
            }
            //addTelemetry();
        }
        addTelemetry();

        setDrive(0);
        sleep(100);

        while (Math.abs(navx.getYaw() + 88) > 2 && opModeIsActive()) {
            int scale;
            if (navx.getYaw() + 88 > 0) {
                scale = -1;
            } else {
                scale = 1;
            }
            setDrive(0, -scale * 0.07);
        }
        addTelemetry();

        setDrive(0);
        sleep(100);

//        setDrive(0, 0, 0, 0);
//        sleep(100);

//        // turn until back color sensor also detects the line
//        while (colorBackReader.read(0x08, 1)[0] < 25) {
////            if (FL.getCurrentPosition() > -7000) { //test encoder value
////                setDrive(-0.1, 0.1, -0.1, 0.1);
////            } else {
//            setDrive(-0.06, 0.06, -0.06, 0.06);
////            }
//        }

//        setDrive(0, 0, 0, 0);
//        sleep(100);

        // drive forward until close enough to beacon
        while (rangeReader.read(0x04, 2)[0] > 15 && opModeIsActive()) {
            setDrive(-0.1);
            addTelemetry();
        }

        setDrive(0);
        sleep(100);

//        setDrive(0, 0, 0, 0);
//        sleep(100);

        // move beacon pusher arm to appropriate location
        if (Math.abs(colorLeftReader.read(0x04, 1)[0] - 10) <= 1) {// && colorRightCache[0] == 3){
            //left is red, right is blue
            swingArm.setPosition(.72);
        }
        else if (Math.abs(colorLeftReader.read(0x04, 1)[0] - 3) <= 1) {// && colorRightCache[0] == 10){
            swingArm.setPosition(.15);
        }
        addTelemetry();

        // drive forward to hit beacon
        while (rangeReader.read(0x04, 2)[1] < 12 && opModeIsActive()) {
            setDrive(-0.15, -0.15, -0.15, -0.15);
            addTelemetry();
        }

//        setDrive(0, 0, 0, 0);
//        sleep(100);

        // back away from beacon
        while (rangeReader.read(0x04, 2)[0] < 20 && opModeIsActive()) {
            setDrive(0.1, 0.1, 0.1, 0.1);
            addTelemetry();
        }

        setDrive(0, 0, 0, 0);
        sleep(100);

        while (Math.abs(navx.getYaw() + 45) > 3 && opModeIsActive()){
            if (Math.abs(navx.getYaw()) > 45) {
                setDrive(0.4, 0, 0.4, 0);
            } else {
                setDrive(0.1, 0, 0.1, 0);
            }
            addTelemetry();
        }

        setDrive(0, 0, 0, 0);
        sleep(100);

//        double currentFLencoder = FL.getCurrentPosition();
//        while (FL.getCurrentPosition() - currentFLencoder < -600){
//            setDrive(.4, .4, .4, .4);
//        }
//
//        setDrive(0, 0, 0, 0);
//        sleep(100);

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

    private void setEnc(int p1, int p2, int p3, int p4) {
        FL.setTargetPosition(FL.getCurrentPosition() + p1);
        FR.setTargetPosition(FR.getCurrentPosition() + p2);
        BL.setTargetPosition(BL.getCurrentPosition() + p3);
        BR.setTargetPosition(BR.getCurrentPosition() + p4);
    }

}