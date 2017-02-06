package org.firstinspires.ftc.rmrobotics.opmodes.feRMilab.autored;

import com.kauailabs.navx.ftc.AHRS;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
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
// RED TEAM


@Autonomous(name = "BeaconCap")
public class BeaconCap extends LinearOpMode {
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

    private I2cDevice range;
    private I2cDeviceSynch rangeReader;

    @Override
    public void runOpMode() {
        // motor initialization
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

        // navx initialization and calibration
        dim = hardwareMap.deviceInterfaceModule.get("dim");
        navx = AHRS.getInstance(dim, 0, AHRS.DeviceDataType.kProcessedData, (byte) 50);
        while (navx.isCalibrating()) {
            telemetry.addData("Status", !navx.isCalibrating());
            telemetry.update();
        }

        // back color sensor
        colorBack = hardwareMap.i2cDevice.get("colorBack");
        colorBackReader = new I2cDeviceSynchImpl(colorBack, I2cAddr.create8bit(0x50), false);
        colorBackReader.engage();
        colorBackReader.write8(3,0);

        // center color sensor
        colorCenter = hardwareMap.i2cDevice.get("colorCenter");
        colorCenterReader = new I2cDeviceSynchImpl(colorCenter, I2cAddr.create8bit(0x52), false);
        colorCenterReader.engage();
        colorCenterReader.write8(3,0);

        // right beacon color sensor
        colorRight = hardwareMap.i2cDevice.get("colorRight");
        colorRightReader = new I2cDeviceSynchImpl(colorRight, I2cAddr.create8bit(0x70), false);
        colorRightReader.engage();
        colorRightReader.write8(3,1);

        // left beacon color sensor
        colorLeft = hardwareMap.i2cDevice.get("colorLeft");
        colorLeftReader = new I2cDeviceSynchImpl(colorLeft, I2cAddr.create8bit(0x72), false);
        colorLeftReader.engage();
        colorLeftReader.write8(3,1);

        // range finder
        range = hardwareMap.i2cDevice.get("range");
        rangeReader = new I2cDeviceSynchImpl(range, I2cAddr.create8bit(0x60), false);
        rangeReader.engage();

        // set LED to alliance color
        dim.setLED(0, false); // blue
        dim.setLED(1, true); // red

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        runtime.reset(); // reset runtime counter
        navx.zeroYaw(); // reset navx yaw value
        swingArm.setPosition(0.5);
        harvester.setPosition(0.5);

        // turn towards first beacon
        setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        turnRobotCorner(37);
        /*while (Math.abs(navx.getYaw() + 37) > 2 && opModeIsActive()) {
            int scale;
            if (navx.getYaw() + 37 > 0) {
                scale = -1;
            } else {
                scale = 1;
            }
            if (Math.abs(navx.getYaw() + 37) > 15) {
                setDrive(scale * 0.4, 0);
            } else {
                setDrive(scale * 0.07, 0);
            }
        }
        addTelemetry();*/
//        setDrive(0, 0, 0, 0);
//        sleep(100);

        // drive forward until center color sensor detects line
        double initPos = Math.abs(FL.getCurrentPosition());
        while (colorCenterReader.read(0x08, 1)[0] < 25 && opModeIsActive()) {
            int scale = -1;
            if (FL.getCurrentPosition() - initPos < -3500) {
                scale = 1;
            } else if (FL.getCurrentPosition() - initPos > -2400) {
                scale = -1;
            }
            if (FL.getCurrentPosition() - initPos > -2400) {
                setDrive(scale * 0.5);
            } else {
                setDrive(scale * 0.1);
            }
        }
        addTelemetry();
        setDrive(0);
        sleep(100);

        // drives backwards to correct for overshooting
        while (colorCenterReader.read(0x08, 1)[0] < 25 && opModeIsActive()) {
            setDrive(0.07);
        }
        addTelemetry();
        setDrive(0, 0, 0, 0);
        sleep(100);

        // turn left until back color sensor also detects the line
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
        }
        addTelemetry();
        setDrive(0, 0, 0, 0);
        sleep(100);

        // turn right to correct overturning
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
        while (rangeReader.read(0x04, 2)[0] > 17 && opModeIsActive()) {
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
        if (Math.abs(colorLeftReader.read(0x04, 1)[0] - 10) <= 1) {// || Math.abs(colorRightReader.read(0x04, 1)[0] - 3) <= 1) {// && colorRightCache[0] == 3){
            //left is red, right is blue
            swingArm.setPosition(.65);
        }
        else if (Math.abs(colorLeftReader.read(0x04, 1)[0] - 3) <= 1) {// || Math.abs(colorRightReader.read(0x04, 1)[0] - 10) <= 1) {// && colorRightCache[0] == 10){
            swingArm.setPosition(.4);
        }
        addTelemetry();


        double initTime = runtime.milliseconds();
        // drive forward to hit beacon
        //while (rangeReader.read(0x04, 2)[1] < 13 && opModeIsActive()) {
        while (runtime.milliseconds() - initTime < 1000 && opModeIsActive()) {
        // drive forward to hit beacon
        //while (rangeReader.read(0x04, 2)[1] < 12 && opModeIsActive()) {
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
        swingArm.setPosition(0.5);


        //FIRST BEACON DONE


//        setDrive(0, 0, 0, 0);
//        sleep(100);

        // turn towards second line
        while (navx.getYaw() < -10 && opModeIsActive()) {
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
            setDrive(-0.6);
            addTelemetry();
        }

        // drive forward until center color sensor detects second line
        while (colorCenterReader.read(0x08, 1)[0] < 25 && opModeIsActive()) {
            int scale = -1;
            int relDis = FL.getCurrentPosition() - curEnc;
            if (relDis > -1500) {
                setDrive(-0.35);
            } else {
                if (relDis > -1000) {
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

        //drive backwards to correct for overshooting
        while (colorCenterReader.read(0x08, 1)[0] < 25 && opModeIsActive()) {
            setDrive(0.04);
        }
        addTelemetry();

        setDrive(0);
        sleep(100);

        //turn left towards beacon
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

        //turn right to correct for overturning
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
            swingArm.setPosition(.68);
        }
        else if (Math.abs(colorLeftReader.read(0x04, 1)[0] - 3) <= 1) {// && colorRightCache[0] == 10){
            swingArm.setPosition(.20);
        }
        addTelemetry();

        initTime = runtime.milliseconds();
        // drive forward to hit beacon
        //while (rangeReader.read(0x04, 2)[1] < 13 && opModeIsActive()) {
        while (runtime.milliseconds() - initTime < 1000 && opModeIsActive()) {
        // drive forward to hit beacon
        //while (rangeReader.read(0x04, 2)[1] < 12 && opModeIsActive()) {
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

        //turn towards center goal
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

    private void turnRobotCorner(int a){
        while (Math.abs(navx.getYaw() + a) > 2 && opModeIsActive()) {
            int scale;
            if (navx.getYaw() + a > 0) {
                //if robot has turned less than a degrees in left direction
                scale = -1;
            } else {
                //if robot has turned more than a degrees in left direction
                scale = 1;
            }
            if (Math.abs(navx.getYaw() + a) > 15){
                //if robot has turned less than (a-15) degrees in either direction
                //then turns robot at a faster speed
                setDrive(scale * 0.4, 0);
            } else {
                //if robot has turned more than (a-15) degrees in either direction
                //turns robot at a slower speed
                setDrive(scale * 0.07, 0);
            }
        }
        addTelemetry();
//        setDrive(0);
//        sleep(100);
    }
}