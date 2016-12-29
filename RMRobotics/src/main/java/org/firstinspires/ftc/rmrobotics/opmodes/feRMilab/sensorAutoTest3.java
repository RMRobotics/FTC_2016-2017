package org.firstinspires.ftc.rmrobotics.opmodes.feRMilab;

import com.kauailabs.navx.ftc.AHRS;
import com.kauailabs.navx.ftc.navXPIDController;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchImpl;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.text.DecimalFormat;

/**
 * Created by Peter on 11/29/16.
 */

@Autonomous(name = "sensors3")
public class sensorAutoTest3 extends OpMode {
    //refer to comments in sensorAutoTest
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

    private double yaw;

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
    public void init() {
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
        FL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
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
    }

    public void init_loop() {
        calibration_complete = !navx.isCalibrating();
        if ( calibration_complete ) {
            navx.zeroYaw();
        }
        telemetry.addData("navx calibrated: ", !navx.isCalibrating());
    }

    @Override
    public void start() {
        navx.zeroYaw();
        yawPIDResult = new navXPIDController.PIDResult();
    }

    @Override
    public void loop() {
        telemetry.addData("Status", "Running: " + runtime.toString());

        colorLinecache = colorLinereader.read(0x04, 1);
        telemetry.addData("1 #L", colorLinecache[0] & 0xFF);
        telemetry.addData("2 A", colorLinereader.getI2cAddress().get8Bit());
        telemetry.addData("3 YAW", navx.getYaw());
        telemetry.addData("enc", FL.getCurrentPosition() + " " + FR.getCurrentPosition() + " " + BL.getCurrentPosition() + " " + BR.getCurrentPosition());

        if (!turn1) {
        //robot isn't turned towards beacon
            if (Math.abs(35 + navx.getYaw()) < 2) {//if robot has turned 35 degrees
                turn1 = true;//change turn1 to true
            } else if (35 + navx.getYaw() > 0) {//if robot has turned left less than 35 degrees
                setDrive(-0.3, 0, -0.3, 0);//turn the robot left
            } else {
                setDrive(0.3, 0, 0.3, 0);//turn the robot right
            }
        } else if (!lineSeen) {
        //if line isn't seen
            setDrive(-0.3, -0.3, -0.3, -0.3);//drive towards beacon
            if (colorLinecache[0] == 14) {//if white color is detected, turn lineSeen to true
                lineSeen = true;
            }
        } else if (!turn2) {
        //if robot isn't aligned with the wall
            if (Math.abs(80 + navx.getYaw()) < 2) {//if robot has turned 80 degrees, set turn2 to true
                turn2 = true;
            } else if (80 + navx.getYaw() > 0) {//if robot has turned left less than 80 degrees
                setDrive(-0.3, 0, -0.3, 0);//turn the robot left
            } else {
                setDrive(0.3, 0, 0.3, 0);//turn the robot right
            }
        } else {
        //stop robot when done
            setDrive(0, 0, 0, 0);
        }
    }

    private void setDrive(double p1, double p2, double p3, double p4) {
        FL.setPower(p1);
        FR.setPower(p2);
        BL.setPower(p3);
        BR.setPower(p4);
    }
}
