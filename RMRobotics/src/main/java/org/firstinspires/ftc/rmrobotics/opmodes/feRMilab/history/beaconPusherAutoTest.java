package org.firstinspires.ftc.rmrobotics.opmodes.feRMilab.history;

import android.util.Log;

import com.kauailabs.navx.ftc.AHRS;
import com.kauailabs.navx.ftc.navXPIDController;
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
 * Created by Simon on 1/13/17.
 */

@Autonomous(name = "beaconPusherAuto")
@Disabled
public class beaconPusherAutoTest extends LinearOpMode{
    private ElapsedTime runtime = new ElapsedTime();

    DcMotor FL;
    DcMotor FR;
    DcMotor BL;
    DcMotor BR;

    Servo swingArm;

    byte[] colorBackCache;
    I2cDevice colorBack;
    I2cDeviceSynch colorBackReader;
    byte[] colorCenterCache;
    I2cDevice colorCenter;
    I2cDeviceSynch colorCenterReader;
    byte[] colorRightCache;
    I2cDevice colorRight;
    I2cDeviceSynch colorRightReader;
    byte[] colorLeftCache;
    I2cDevice colorLeft;
    I2cDeviceSynch colorLeftReader;

    byte[] rangeCache;
    I2cDevice range;
    I2cDeviceSynch rangeReader;
    int LUS;
    int LODS;
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
        setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        telemetry.addData("init", "pre");
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

        telemetry.addData("init", "final");
        telemetry.update();

        waitForStart();

        //navx.zeroYaw();
        sensorUpdate();
        addTelemetry();
        //0x04 is color number
//        int LUS = rangeCache[0] & 0xFF; //first byte: ultrasonic reading
//        int LODS = rangeCache[1] & 0xFF; //second byte: optical reading

        while (LUS > 13) {
//            double steer = 0;
//            //double steer = (navx.getYaw()) * 0.01;
//            if (LUS > 13) {
//                setDrive(-0.2 - steer, -0.2 - steer, -0.2, -0.2);
//            } else {
//                setDrive(-0.1 - steer/2, -0.1 - steer/2, -0.1, -0.1);
//            }
//            sensorUpdate();
            setDrive(-0.1, -0.1, -0.1, -0.1);
            sensorUpdate();
            addTelemetry();
        }
        //drive forward until close to color beacon

        //2 is blue and 11 is red
//        while ((colorLeftCache[0] != 3 || colorLeftCache[0] != 10) || (colorRightCache[0] != 3) || (colorRightCache[0] != 10)){
//            addTelemetry();
//            sensorUpdate();
//        }

        //swingArm.scaleRange(-1,1);

        //sensorUpdate();
        //.72 for left, .15 for right
        setDrive(0, 0, 0, 0);
        double startTime = runtime.milliseconds();
        while (runtime.milliseconds() - startTime < 7500) {
            addTelemetry();
        }
        addTelemetry();
//        while ((colorLeftCache[0] == 3 || colorLeftCache[0] == 10) && (colorRightCache[0] == 3 || colorRightCache[0] == 10)) {
//            if (colorLeftCache[0] == 10 && colorRightCache[0] == 3){
//                //left is red, right is blue
//                swingArm.setPosition(.72);
//            }
//            else if (colorLeftCache[0] == 3 && colorRightCache[0] == 10){
//                swingArm.setPosition(.15);
//            }
//        }
        if (colorLeftCache[0] == 10 && colorRightCache[0] == 3){
            //left is red, right is blue
            swingArm.setPosition(.72);
        }
        else if (colorLeftCache[0] == 3 && colorRightCache[0] == 10){
            swingArm.setPosition(.15);
        }
        addTelemetry();

        sensorUpdate();
        while (LODS < 33) {
            setDrive(-0.15, -0.15, -0.15, -0.15);
            sensorUpdate();
            addTelemetry();
        }

        sleep(500);

        sensorUpdate();
        while (LUS < 20) {
            setDrive(0.3, 0.3, 0.3, 0.3);
            sensorUpdate();
            addTelemetry();
        }
    }
    private void sensorUpdate() {
        colorCenterCache = colorCenterReader.read(0x08, 1);
        colorBackCache = colorBackReader.read(0x08, 1);
        colorLeftCache = colorLeftReader.read(0x04, 1);
        colorRightCache = colorRightReader.read(0x04, 1);
        rangeCache = rangeReader.read(0x04, 2);
        LUS = rangeCache[0] & 0xFF;
        LODS = rangeCache[1] & 0xFF;
    }

    private void addTelemetry() {
        telemetry.addData("1 Time", runtime.seconds());
        //telemetry.addData("2 Yaw", navx.getYaw());
        telemetry.addData("3 ColorBack", colorBackCache[0] & 0xFF);
        telemetry.addData("4 ColorCenter", colorCenterCache[0] & 0xFF);
        telemetry.addData("5 ColorLeft", colorLeftCache[0] & 0xFF);
        telemetry.addData("6 ColorRight", colorRightCache[0] & 0xFF);
        telemetry.addData("7 Range", rangeCache[0] + " " + rangeCache[1]);
        telemetry.addData("8 Motor", FL.getPower() + " " + FR.getPower() + " " + BL.getPower() + " " + BR.getPower());
        telemetry.addData("9 Encoder", FL.getCurrentPosition() + " " + FR.getCurrentPosition() + " " + BL.getCurrentPosition() + " " + BR.getCurrentPosition());
        telemetry.update();
    }

    private void addLog() {
        Log.d("1 Time", String.valueOf(runtime.seconds()));
        //Log.d("2 Yaw", String.valueOf(navx.getYaw()));
        Log.d("3 ColorBack", String.valueOf(colorBackCache[0] & 0xFF));
        Log.d("4 ColorCenter", String.valueOf(colorCenterCache[0] & 0xFF));
        Log.d("5 ColorLeft", String.valueOf(colorLeftCache[0] & 0xFF));
        Log.d("6 ColorRight", String.valueOf(colorRightCache[0] & 0xFF));
        Log.d("7 Range", rangeCache[0] + " " + rangeCache[1]);
        Log.d("8 Motor", FL.getPower() + " " + FR.getPower() + " " + BL.getPower() + " " + BR.getPower());
        Log.d("9 Encoder", FL.getCurrentPosition() + " " + FR.getCurrentPosition() + " " + BL.getCurrentPosition() + " " + BR.getCurrentPosition());
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
