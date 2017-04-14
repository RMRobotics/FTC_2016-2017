package org.firstinspires.ftc.rmrobotics.histoy.opmodes.feRMilab;

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
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Created by Simon on 12/30/16.
 */

@Autonomous(name = "sensors5")
@Disabled
public class sensorAutoLinear extends LinearOpMode {
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
    navXPIDController.PIDResult yawPIDResult;

    byte[] colorLineCache;
    I2cDevice colorLine;
    I2cDeviceSynch colorLineReader;

    byte[] rangeCache;

    I2cDevice range;
    I2cDeviceSynch rangeReader;

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

        colorLine = hardwareMap.i2cDevice.get("colorBack");
        colorLineReader = new I2cDeviceSynchImpl(colorLine, I2cAddr.create8bit(0x50), false);
        colorLineReader.engage();
        colorLineReader.write8(3,0);

        range = hardwareMap.i2cDevice.get("range");
        rangeReader = new I2cDeviceSynchImpl(range, I2cAddr.create8bit(0x60), false);
        rangeReader.engage();

        waitForStart();

        navx.zeroYaw();
        yawPIDResult = new navXPIDController.PIDResult();

        setEnc(-560, -560, -560, -560);
        setDrive(0.3, 0.3, 0.3, 0.3);
        sleep(2000);

        setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        colorLineCache = colorLineReader.read(0x04, 1);
        while (opModeIsActive() && Math.abs(navx.getYaw() + 50) > 1) {
            double power = -((navx.getYaw() + 50) / 30) * 0.3;
            if (Math.abs(power) > 0.3) {
                power /= Math.abs(power);
                power *= 0.3;
            }
            setDrive(power, 0, power, 0);
            addTelemetry();
        }

        colorLineCache = colorLineReader.read(0x04, 1);
        while (opModeIsActive() && colorLineCache[0] != 14) {
            telemetry.addData("color", colorLineCache[0]);
            if (FL.getCurrentPosition() > -3800) {
                setDrive(-0.3, -0.3, -0.3, -0.3);
            } else {
                setDrive(-0.12, -0.12, -0.12, -0.12);
            }
            addTelemetry();
            colorLineCache = colorLineReader.read(0x04, 1);
        }

        setZeroMode(DcMotor.ZeroPowerBehavior.FLOAT);
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

        setMode(DcMotor.RunMode.RUN_TO_POSITION);
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
        sleep(1500);

        setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        colorLineCache = colorLineReader.read(0x04, 1);
        while (opModeIsActive() && colorLineCache[0] == 14) {
            setDrive(-0.3, 0.3, 0.3, -0.3);
            colorLineCache = colorLineReader.read(0x04, 1);
            addTelemetry();
        }

        sleep(100);

        while (opModeIsActive() && colorLineCache[0] != 14) {
            setDrive(-0.3, 0.3, 0.3, -0.3);
            addTelemetry();
            colorLineCache = colorLineReader.read(0x04, 1);
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

    private void addTelemetry() {
        telemetry.addData("1 Time", runtime.seconds());
        telemetry.addData("2 Yaw", navx.getYaw());
        telemetry.addData("3 Color", colorLineCache[0] & 0xFF);
        telemetry.addData("4 Motor", FL.getPower() + " " + FR.getPower() + " " + BL.getPower() + " " + BR.getPower());
        telemetry.addData("5 Encoder", FL.getCurrentPosition() + " " + FR.getCurrentPosition() + " " + BL.getCurrentPosition() + " " + BR.getCurrentPosition());
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
