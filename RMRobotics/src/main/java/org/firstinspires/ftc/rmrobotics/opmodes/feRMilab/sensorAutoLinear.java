package org.firstinspires.ftc.rmrobotics.opmodes.feRMilab;

import com.kauailabs.navx.ftc.AHRS;
import com.kauailabs.navx.ftc.navXPIDController;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchImpl;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.io.InterruptedIOException;
import java.text.DecimalFormat;

/**
 * Created by Simon on 12/30/16.
 */

@Autonomous(name = "sensor5")
public class sensorAutoLinear extends LinearOpMode {
    DcMotor FL;
    DcMotor FR;
    DcMotor BL;
    DcMotor BR;
    private final int NAVX_DIM_I2C_PORT = 0;
    private AHRS navx;
    private navXPIDController yawPIDController;
    private ElapsedTime runtime = new ElapsedTime();

    private final byte NAVX_DEVICE_UPDATE_RATE_HZ = 50;

    private final double TARGET_ANGLE_DEGREES = 0.0;
    private final double TOLERANCE_DEGREES = 1.5;
    private final double MIN_MOTOR_OUTPUT_VALUE = -0.3;
    private final double MAX_MOTOR_OUTPUT_VALUE = 0.3;
    private final double YAW_PID_P = 0.03;
    private final double YAW_PID_I = 0.0;
    private final double YAW_PID_D = 0.0;

    private boolean calibration_complete = false;
    private boolean lineSeen = false;
    private boolean turn1 = false;//turn towards beacon
    private boolean turn2 = false;//turn towards wall

    navXPIDController.PIDResult yawPIDResult;
    DecimalFormat df;
    byte[] colorLinecache;
    I2cDevice colorLine;
    I2cDeviceSynch colorLinereader;

    @Override
    public void runOpMode() throws InterruptedException {
        FL = hardwareMap.dcMotor.get("FL");
        FR = hardwareMap.dcMotor.get("FR");
        BL = hardwareMap.dcMotor.get("BL");
        BR = hardwareMap.dcMotor.get("BR");
        FR.setDirection(DcMotor.Direction.REVERSE);
        BR.setDirection(DcMotor.Direction.REVERSE);
        FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
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

        df = new DecimalFormat("#.##");
        telemetry.addData("Status", "Initialized");
        colorLine = hardwareMap.i2cDevice.get("colorLine");
        colorLinereader = new I2cDeviceSynchImpl(colorLine, I2cAddr.create8bit(0x50), false);
        colorLinereader.engage();
        colorLinereader.write8(3,0);

        waitForStart();

        navx.zeroYaw();
        yawPIDResult = new navXPIDController.PIDResult();

        FL.setTargetPosition(-560);
        FR.setTargetPosition(-560);
        BL.setTargetPosition(-560);
        BR.setTargetPosition(-560);
        setDrive(0.3,0.3,0.3,0.3);
        sleep(2000);

        FL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        while (Math.abs(navx.getYaw() + 45) > 1) {
            double power = -((navx.getYaw() + 45) / 20) * 0.3;
            if (Math.abs(power) > 0.3) {
                power /= Math.abs(power);
                power *= 0.3;
            }
            setDrive(power, 0, power, 0);
            telemetry.addData("yaw", navx.getYaw());
            telemetry.update();
        }

        colorLinecache = colorLinereader.read(0x04, 1);
        while (colorLinecache[0] != 14) {
            if (FL.getCurrentPosition() > -3800) {
                setDrive(-0.3, -0.3, -0.3, -0.3);
            } else {
                setDrive(-0.12, -0.12, -0.12, -0.12);
            }
            telemetry.addData("encoders", FL.getCurrentPosition() + " " + FR.getCurrentPosition() + " " + BL.getCurrentPosition() + " " + BR.getCurrentPosition());
            telemetry.update();
            colorLinecache = colorLinereader.read(0x04, 1);
        }

        while (Math.abs(navx.getYaw() + 90) > 1) {
            BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            setDrive(-0.2, 0.2, -0.07, 0.07);
            telemetry.addData("encoders", FL.getCurrentPosition() + " " + FR.getCurrentPosition() + " " + BL.getCurrentPosition() + " " + BR.getCurrentPosition());
            telemetry.update();
        }

        BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        setDrive(0, 0, 0, 0);

        /*
        colorLinecache = colorLinereader.read(0x04, 1);
        while (colorLinecache[0] != 14) {
            setDrive(0.2, -0.2, -0.2, 0.2);
            colorLinecache = colorLinereader.read(0x04, 1);
        }
        */
        while (runtime.seconds() < 400000) {
            telemetry.update();
        }
    }

    private void setDrive(double p1, double p2, double p3, double p4) {
        FL.setPower(p1);
        FR.setPower(p2);
        BL.setPower(p3);
        BR.setPower(p4);
    }
}
