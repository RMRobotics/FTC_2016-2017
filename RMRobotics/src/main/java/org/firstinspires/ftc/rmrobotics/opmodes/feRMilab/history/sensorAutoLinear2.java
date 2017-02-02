package org.firstinspires.ftc.rmrobotics.opmodes.feRMilab.history;

import android.os.Looper;
import android.util.Log;

import com.kauailabs.navx.ftc.AHRS;
import com.kauailabs.navx.ftc.navXPIDController;
import com.qualcomm.hardware.adafruit.BNO055IMU;
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


@Autonomous(name = "sensors6")
@Disabled
public class sensorAutoLinear2 extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();

    DcMotor FL;
    DcMotor FR;
    DcMotor BL;
    DcMotor BR;

    Servo swingArm;

    private final int NAVX_DIM_I2C_PORT = 0;
    private AHRS navx;
//    private navXPIDController yawPIDController;
    private final byte NAVX_DEVICE_UPDATE_RATE_HZ = 50;
//    private final double TARGET_ANGLE_DEGREES = 0.0;
//    private final double TOLERANCE_DEGREES = 1.5;
//    private final double MIN_MOTOR_OUTPUT_VALUE = -0.3;
//    private final double MAX_MOTOR_OUTPUT_VALUE = 0.3;
//    private final double YAW_PID_P = 0.03;
//    private final double YAW_PID_I = 0.0;
//    private final double YAW_PID_D = 0.0;
//    //navXPIDController.PIDResult yawPIDResult;

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
        setMode(DcMotor.RunMode.RUN_TO_POSITION);

        navx = AHRS.getInstance(hardwareMap.deviceInterfaceModule.get("dim"),
                NAVX_DIM_I2C_PORT,
                AHRS.DeviceDataType.kProcessedData,
                NAVX_DEVICE_UPDATE_RATE_HZ);
//        yawPIDController = new navXPIDController( navx, navXPIDController.navXTimestampedDataSource.YAW);
//        yawPIDController.setSetpoint(TARGET_ANGLE_DEGREES);
//        yawPIDController.setContinuous(true);
//        yawPIDController.setOutputRange(MIN_MOTOR_OUTPUT_VALUE, MAX_MOTOR_OUTPUT_VALUE);
//        yawPIDController.setTolerance(navXPIDController.ToleranceType.ABSOLUTE, TOLERANCE_DEGREES);
//        yawPIDController.setPID(YAW_PID_P, YAW_PID_I, YAW_PID_D);
//        yawPIDController.enable(true);

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

        waitForStart();
        runtime.reset();
        navx.zeroYaw();
        //yawPIDResult = new navXPIDController.PIDResult();

        setEnc(-560, -560, -560, -560);
        setDrive(0.1, 0.1, 0.1, 0.1);
        sleep(150);
        setDrive(0.3, 0.3, 0.3, 0.3);
        sleep(800);
        setDrive(0.1, 0.1, 0.1, 0.1);
        sleep(150);
        //goes forward slow, fast, slow

        sensorUpdate();
        setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        while (Math.abs(navx.getYaw() + 45) > 3) {
            if (Math.abs(navx.getYaw()) < 35) {
                setDrive(-0.2, 0, -0.2, 0);
            } else {
                setDrive(-0.1, 0, -0.1, 0);
            }
//            double power = -(((navx.getYaw() + 45) / 30) * 0.3);
//            if (Math.abs(power) > 0.35) {
//                power /= Math.abs(power);
//                power *= 0.3;
//            }
//            setDrive(power, 0, power, 0);
            addTelemetry();
            //turns robot towards first beacon
        }
        //turns robot towards first beacon


        sensorUpdate();
        //while (colorCenterReader.read(0x08, 1)[0] < 25) {
        while (colorCenterReader.read(0x08, 1)[0] < 25) {
//            if (!tripped) {
//                if (FL.getCurrentPosition() > -3500) {
//                    setDrive(-0.3, -0.3, -0.3, -0.3);
//                } else {
//                    setDrive(-0.2, -0.2, -0.2, -0.2);
//                }
//            } else {
//                setDrive(-0.1, -0.1, -0.1, -0.1);
//            }
//            if (colorBackReader.read(0x08, 1)[0] > 14) {
//                tripped = true;
//            }
            if (FL.getCurrentPosition() > -4200) {
                setDrive(-0.3, -0.3, -0.3, -0.3);
            } else {
                setDrive(-0.06, -0.06, -0.06, -0.06);
            }
//            addTelemetry();
//            sensorUpdate();
//            if (colorBackReader.read(0x08, 1)[0] > 25) {
//                tripped = true;
//            }
        }
        addTelemetry();
        setZeroMode(DcMotor.ZeroPowerBehavior.BRAKE);
        setDrive(0, 0, 0, 0);
        //drives until it senses white line

        sensorUpdate();
        while (colorBackReader.read(0x08, 1)[0] < 25) {
            if (FL.getCurrentPosition() > -4600) {
                setDrive(-0.1, 0.1, -0.1, 0.1);
            } else {
                setDrive(-0.06, 0.06, -0.06, 0.06);
            }
            //addTelemetry();
            //sensorUpdate();
        }
        setZeroMode(DcMotor.ZeroPowerBehavior.BRAKE);
        setDrive(0, 0, 0, 0);
//        while (colorBackReader.read(0x08, 1)[0] < 25) {
//            if (navx.getYaw() + 70 > 0) {
//                setDrive(-0.15, 0.15, -0.15, 0.15);
//            } else {
//                setDrive(-0.08, 0.08, -0.08, 0.08);
//            }
//            addTelemetry();
//            sensorUpdate();
//        }

//        setZeroMode(DcMotor.ZeroPowerBehavior.FLOAT);
//        while (Math.abs(navx.getYaw() + 90) > 1) {
//            if (navx.getYaw() + 90 > 10) {
//                setDrive(-0.2, 0.2, -0.2, 0.2);
//            } else if (navx.getYaw() + 90 < -10) {
//                setDrive(0.2, -0.2, 0.2, -0.2);
//            } else if (navx.getYaw() + 90 <= 10 && navx.getYaw() + 90 > 0) {
//                setDrive(-0.1, 0.1, -0.1, 0.1);
//            } else if (navx.getYaw() + 90 >= -10 && navx.getYaw() + 90 < 0) {
//                setDrive(0.1, -0.1, 0.1, -0.1);
//            } else {
//                setDrive(0, 0, 0, 0);
//            }
//            addTelemetry();
//            //aligns with wall
//        }
//        //aligns with wall

        /*while (Math.abs(navx.getYaw() + 90) > 1) {
            if (navx.getYaw() + 90 > 10) {
                setDrive(-0.2, 0.2, -0.07, 0.07);
            } else if (navx.getYaw() + 90 < -10) {
                setDrive(0.2, -0.2, 0.07, -0.07);
            } else if (navx.getYaw() + 90 <= 10 && navx.getYaw() + 90 > 0) {
                setDrive(-0.1, 0.1, -0.035, 0.035);
            } else if (navx.getYaw() + 90 >= -10 && navx.getYaw() + 90 < 0) {
                setDrive(0.1, -0.1, 0.035, -0.035);
            } else {
                setDrive(0, 0, 0, 0);
            }
            addTelemetry();
        }*/
        //different values for aligning with wall
        sensorUpdate();
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
        while (runtime.milliseconds() - startTime < 2500) {
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
        if (colorLeftCache[0] == 10) {// && colorRightCache[0] == 3){
            //left is red, right is blue
            swingArm.setPosition(.72);
        }
        else if (colorLeftCache[0] == 3) {// && colorRightCache[0] == 10){
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

        addTelemetry();
        sensorUpdate();
        while (LUS < 20) {
            setDrive(0.1, 0.1, 0.1, 0.1);
            sensorUpdate();
            addTelemetry();
        }

        setZeroMode(DcMotor.ZeroPowerBehavior.BRAKE);
        setDrive(0, 0, 0, 0);
        //drive back to drive to next beacon

        sensorUpdate();
        while (Math.abs(navx.getYaw()) > 2) {
            if (Math.abs(navx.getYaw()) > 15) {
                setDrive(0.2, -0.2, 0.2, -0.2);
            } else {
                setDrive(0.07, -0.07, 0.07, -0.07);
            }
//            double power = -(((navx.getYaw() + 45) / 30) * 0.3);
//            if (Math.abs(power) > 0.35) {
//                power /= Math.abs(power);
//                power *= 0.3;
//            }
//            setDrive(power, 0, power, 0);
            addTelemetry();
            //turns robot towards first beacon
        }

        double start = runtime.milliseconds();
        while (runtime.milliseconds() - start < 1000) {
            setDrive(-0.07, -0.07, -0.07, -0.07);
        }
        //while (colorCenterReader.read(0x08, 1)[0] < 25) {
        while (colorCenterReader.read(0x08, 1)[0] < 15) {
//            if (!tripped) {
//                if (FL.getCurrentPosition() > -3500) {
//                    setDrive(-0.3, -0.3, -0.3, -0.3);
//                } else {
//                    setDrive(-0.2, -0.2, -0.2, -0.2);
//                }
//            } else {
//                setDrive(-0.1, -0.1, -0.1, -0.1);
//            }
//            if (colorBackReader.read(0x08, 1)[0] > 14) {
//                tripped = true;
//            }
            if (FL.getCurrentPosition() > -6400) {
                setDrive(-0.3, -0.3, -0.3, -0.3);
            } else {
                setDrive(-0.06, -0.06, -0.06, -0.06);
            }
            //sensorUpdate();
            //addTelemetry();
//            addTelemetry();
//            sensorUpdate();
//            if (colorBackReader.read(0x08, 1)[0] > 25) {
//                tripped = true;
//            }
        }
        sensorUpdate();
        addTelemetry();
        setZeroMode(DcMotor.ZeroPowerBehavior.BRAKE);
        setDrive(0, 0, 0, 0);

        /*setMode(DcMotor.RunMode.RUN_TO_POSITION);
        setEnc(FL.getCurrentPosition() - 450,
                FR.getCurrentPosition() - 450,
                BL.getCurrentPosition() - 450,
                BR.getCurrentPosition() - 450);
        setDrive(0.15, 0.15, 0.15, 0.15);
        sleep(1500);
        setEnc(FL.getCurrentPosition() + 450,
                FR.getCurrentPosition() + 450,
                BL.getCurrentPosition() + 450,
                BR.getCurrentPosition() + 450);
        sleep(1500);*/

        //setMode(DcMotor.RunMode.RUN_USING_ENCODER);

//        sensorUpdate();
//        while (colorBackCache[0] == 14) {
//            setDrive(-0.3, 0.3, 0.3, -0.3);
//            sensorUpdate();
//            addTelemetry();
//        }
//
//        sleep(100);

//        while (colorBackCache[0] != 14) {
//            setDrive(-0.3, 0.3, 0.3, -0.3);
//            addTelemetry();
//            sensorUpdate();
//        }

//        while (Math.abs(navx.getYaw() + 90) > 1) {
//            if (navx.getYaw() + 90 > 10) {
//                setDrive(-0.2, 0.2, -0.07, 0.07);
//            } else if (navx.getYaw() + 90 < -10) {
//                setDrive(0.2, -0.2, 0.07, -0.07);
//            } else if (navx.getYaw() + 90 <= 10 && navx.getYaw() + 90 > 0) {
//                setDrive(-0.1, 0.1, -0.035, 0.035);
//            } else if (navx.getYaw() + 90 >= -10 && navx.getYaw() + 90 < 0) {
//                setDrive(0.1, -0.1, 0.035, -0.035);
//            } else {
//                setDrive(0, 0, 0, 0);
//            }
//            addTelemetry();
//        }

        setDrive(0, 0, 0, 0);

        stop();
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
        telemetry.addData("2 Yaw", navx.getYaw());
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
        Log.d("2 Yaw", String.valueOf(navx.getYaw()));
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