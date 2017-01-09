package org.firstinspires.ftc.rmrobotics.opmodes.feRMilab;

import com.kauailabs.navx.ftc.AHRS;
import com.kauailabs.navx.ftc.navXPIDController;
import com.qualcomm.hardware.adafruit.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchImpl;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Created by Memers on 1/6/16.
 */

@Autonomous(name = "sensors6")
public class sensorAutoLinear2 extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();

    DcMotor FL;
    DcMotor FR;
    DcMotor BL;
    DcMotor BR;

    private final int NAVX_DIM_I2C_PORT = 0;
    private AHRS navx;
    private navXPIDController yawPIDController;
    private final byte NAVX_DEVICE_UPDATE_RATE_HZ = 50;
    private final double TARGET_ANGLE_DEGREES = 0.0;
    private final double TOLERANCE_DEGREES = 1.5;
    private final double MIN_MOTOR_OUTPUT_VALUE = -0.3;
    private final double MAX_MOTOR_OUTPUT_VALUE = 0.3;
    private final double YAW_PID_P = 0.03;
    private final double YAW_PID_I = 0.0;
    private final double YAW_PID_D = 0.0;
    //navXPIDController.PIDResult yawPIDResult;

    byte[] colorBackCache;
    I2cDevice colorBack;
    I2cDeviceSynch colorBackReader;
    byte[] colorCenterCache;
    I2cDevice colorCenter;
    I2cDeviceSynch colorCenterReader;

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
        FR.setDirection(DcMotor.Direction.REVERSE);
        BR.setDirection(DcMotor.Direction.REVERSE);
        setZeroMode(DcMotor.ZeroPowerBehavior.BRAKE);
        setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setMode(DcMotor.RunMode.RUN_TO_POSITION);

        navx = AHRS.getInstance(hardwareMap.deviceInterfaceModule.get("dim"),
                NAVX_DIM_I2C_PORT,
                AHRS.DeviceDataType.kProcessedData,
                NAVX_DEVICE_UPDATE_RATE_HZ);
        yawPIDController = new navXPIDController( navx, navXPIDController.navXTimestampedDataSource.YAW);
        yawPIDController.setSetpoint(TARGET_ANGLE_DEGREES);
        yawPIDController.setContinuous(true);
        yawPIDController.setOutputRange(MIN_MOTOR_OUTPUT_VALUE, MAX_MOTOR_OUTPUT_VALUE);
        yawPIDController.setTolerance(navXPIDController.ToleranceType.ABSOLUTE, TOLERANCE_DEGREES);
        yawPIDController.setPID(YAW_PID_P, YAW_PID_I, YAW_PID_D);
        yawPIDController.enable(true);

        while (navx.isCalibrating()) {
            telemetry.addData("Status", !navx.isCalibrating());
        }
        telemetry.addData("Status", "Initialized");

        colorBack = hardwareMap.i2cDevice.get("colorBack");
        colorBackReader = new I2cDeviceSynchImpl(colorBack, I2cAddr.create8bit(0x50), false);
        colorBackReader.engage();
        colorBackReader.write8(3,0);
        colorCenter = hardwareMap.i2cDevice.get("colorCenter");
        colorCenterReader = new I2cDeviceSynchImpl(colorCenter, I2cAddr.create8bit(0x52), false);
        colorCenterReader.engage();
        colorCenterReader.write8(3,0);

        range = hardwareMap.i2cDevice.get("range");
        rangeReader = new I2cDeviceSynchImpl(range, I2cAddr.create8bit(0x60), false);
        rangeReader.engage();

        waitForStart();

        navx.zeroYaw();
        //yawPIDResult = new navXPIDController.PIDResult();

        setEnc(-560, -560, -560, -560);
        setDrive(0.1, 0.1, 0.1, 0.1);
        sleep(150);
        setDrive(0.3, 0.3, 0.3, 0.3);
        sleep(800);
        setDrive(0.1, 0.1, 0.1, 0.1);
        sleep(150);
        //goes forward slow,fast,slow for a little while

        sensorUpdate();
        setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        while (opModeIsActive() && Math.abs(navx.getYaw() + 45) > 1) {
            double power = -(((navx.getYaw() + 45) / 30) * 0.3);
            if (Math.abs(power) > 0.35) {
                power /= Math.abs(power);
                power *= 0.3;
            }
            setDrive(power, 0, power, 0);
            addTelemetry();
        }
        //turns robot towards first beacon

        sensorUpdate();
        while (opModeIsActive() && colorCenterCache[0] != 14) {
            telemetry.addData("color", colorCenterCache[0]);
            if (FL.getCurrentPosition() > -4000) {
                setDrive(-0.3, -0.3, -0.3, -0.3);
            } else {
                setDrive(-0.12, -0.12, -0.12, -0.12);
            }
            addTelemetry();
            sensorUpdate();
        }
        //drives until it senses white line

        setZeroMode(DcMotor.ZeroPowerBehavior.FLOAT);
        while (opModeIsActive() && Math.abs(navx.getYaw() + 90) > 1) {
            if (navx.getYaw() + 90 > 10) {
                setDrive(-0.2, 0.2, -0.2, 0.2);
            } else if (navx.getYaw() + 90 < -10) {
                setDrive(0.2, -0.2, 0.2, -0.2);
            } else if (navx.getYaw() + 90 <= 10 && navx.getYaw() + 90 > 0) {
                setDrive(-0.1, 0.1, -0.1, 0.1);
            } else if (navx.getYaw() + 90 >= -10 && navx.getYaw() + 90 < 0) {
                setDrive(0.1, -0.1, 0.1, -0.1);
            } else {
                setDrive(0, 0, 0, 0);
            }
            addTelemetry();
        }
        //aligns with wall

        /*while (opModeIsActive() && Math.abs(navx.getYaw() + 90) > 1) {
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

        rangeCache = rangeReader.read(0x04, 2);  //Read 2 bytes starting at 0x04
        //0x04 is color number
        int LUS = rangeCache[0] & 0xFF;//first byte: ultrasonic reading
        int LODS = rangeCache[1] & 0xFF;//second byte: optical reading

        while (LUS > 7 && LODS < 4) {
            if (LUS > 20 ) {
                setDrive(-0.4, -0.4, -0.4, -0.4);
            } else if (LUS > 13) {
                setDrive(-0.2, -0.2, -0.2, -0.2);
            } else {
                setDrive(-0.1, -0.1, -0.1, -0.1);
            }
        }
        //drive forward until close to color beacon

        while (LUS < 20) {
            setDrive(0.3, 0.3, 0.3, 0.3);
        }
        //drive back to drive to next beacon

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

        colorBackCache = colorBackReader.read(0x04, 1);
        while (opModeIsActive() && colorBackCache[0] == 14) {
            setDrive(-0.3, 0.3, 0.3, -0.3);
            colorBackCache = colorBackReader.read(0x04, 1);
            addTelemetry();
        }

        sleep(100);

        while (opModeIsActive() && colorBackCache[0] != 14) {
            setDrive(-0.3, 0.3, 0.3, -0.3);
            addTelemetry();
            colorBackCache = colorBackReader.read(0x04, 1);
        }

        while (opModeIsActive() && Math.abs(navx.getYaw() + 90) > 1) {
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
        }

        setDrive(0, 0, 0, 0);

        stop();
    }

    private void sensorUpdate() {
        colorCenterCache = colorCenterReader.read(0x04, 1);
        colorBackCache = colorBackReader.read(0x04, 1);
        rangeCache = rangeReader.read(0x04, 2);
        LUS = rangeCache[0] & 0xFF;
        LODS = rangeCache[1] & 0xFF;
    }

    private void addTelemetry() {
        telemetry.addData("1 Time", runtime.seconds());
        telemetry.addData("2 Yaw", navx.getYaw());
        telemetry.addData("3 Color", colorBackCache[0] & 0xFF);
        telemetry.addData("4 Range", rangeCache[0] + " " + rangeCache[1]);
        telemetry.addData("5 Motor", FL.getPower() + " " + FR.getPower() + " " + BL.getPower() + " " + BR.getPower());
        telemetry.addData("6 Encoder", FL.getCurrentPosition() + " " + FR.getCurrentPosition() + " " + BL.getCurrentPosition() + " " + BR.getCurrentPosition());
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
        FL.setTargetPosition(p1);
        FR.setTargetPosition(p2);
        BL.setTargetPosition(p3);
        BR.setTargetPosition(p4);
    }

}