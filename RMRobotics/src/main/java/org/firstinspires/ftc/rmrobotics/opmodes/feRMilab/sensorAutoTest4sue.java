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
 * Created by RM Robotics on 12/29/2016.
 */
@Autonomous(name = "sensors4")
public class sensorAutoTest4sue extends OpMode {
    private State status;
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

    private double timeStart;

    private boolean calibration_complete = false;
    private Beacon target;

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
        colorLine = hardwareMap.i2cDevice.get("colorBack");
        colorLinereader = new I2cDeviceSynchImpl(colorLine, I2cAddr.create8bit(0x50), false);
        colorLinereader.engage();
        colorLinereader.write8(3,0);
    }

    @Override
    public void start() {
        navx.zeroYaw();
        yawPIDResult = new navXPIDController.PIDResult();
        status = State.DRIVE_1;
        target = Beacon.ONE;
    }

    @Override
    public void loop() {
        telemetry.addData("Status", "Running: " + runtime.toString());

        colorLinecache = colorLinereader.read(0x04, 1);
        telemetry.addData("1 #L", colorLinecache[0] & 0xFF);
        telemetry.addData("2 A", colorLinereader.getI2cAddress().get8Bit());
        telemetry.addData("3 YAW", navx.getYaw());
        telemetry.addData("enc", FL.getCurrentPosition() + " " + FR.getCurrentPosition() + " " + BL.getCurrentPosition() + " " + BR.getCurrentPosition());

        switch (status) {
            case DRIVE_1:
                FL.setTargetPosition(-560);
                FR.setTargetPosition(-560);
                BL.setTargetPosition(-560);
                BR.setTargetPosition(-560);
                setDrive(0.5,0.5,0.5,0.5);
                if(Math.abs(FL.getCurrentPosition() - FL.getTargetPosition()) < 10) {
                    status = State.SHOOT;
                }
                break;
            case SHOOT:
                status = State.TURN_1;
                break;
            case TURN_1:
                FL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                FR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                BL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                BR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                if (Math.abs(navx.getYaw() + 45) < 2) {
                    status = State.DRIVE_2;
                    timeStart = runtime.milliseconds();
                } else {
                    double power = -(navx.getYaw() + 45) / 25;
                    if (Math.abs(power) > 0.5) {
                        power /= 2*Math.abs(power);
                    }
                    setDrive(power, 0, power, 0);
                    telemetry.addData("power", power);
                }
                break;
            case ALIGN_1:
                if (runtime.milliseconds() - timeStart < 10000) {
                    double power = -(navx.getYaw() + 45) / 25;
                    if (Math.abs(power) > 0.4) {
                        power /= 2*Math.abs(power);
                    }
                    setDrive(power, 0, power, 0);
                    telemetry.addData("power", power);
                } else {
                    status = State.DRIVE_2;
                }
                break;
            case DRIVE_2:
                setDrive(-0.3, -0.3, -0.3, -0.3);//drive towards beacon
                if (colorLinecache[0] == 14) {//if white color is detected, turn lineSeen to true
                    status = State.ALIGN_2;
                }
                break;
            case ALIGN_2:
                if (Math.abs(80 + navx.getYaw()) < 2) {//if robot has turned 80 degrees, set turn2 to true
                    status = State.PUSH;
                } else if (80 + navx.getYaw() > 0) {//if robot has turned left less than 80 degrees
                    setDrive(-0.3, 0, -0.3, 0);//turn the robot left
                } else {
                    setDrive(0.3, 0, 0.3, 0);//turn the robot right
                }
                break;
            case PUSH:
                //Once end condition is reached
                if(target == Beacon.ONE){
                    target = Beacon.TwO;
                    status = State.STRAFE;
                }else if(target == Beacon.TwO){
                    status = State.TURN_2;
                }

                break;
            case STRAFE:
                status = State.PUSH;
                break;
            case TURN_2:
                status = State.DRIVE_3;
                break;
            case DRIVE_3:
                status = State.END;
                break;
            case END:
                setDrive(0, 0, 0, 0);
                break;
        }

    }

    private void setDrive(double p1, double p2, double p3, double p4) {
        FL.setPower(p1);
        FR.setPower(p2);
        BL.setPower(p3);
        BR.setPower(p4);
    }
}

enum State {
    DRIVE_1, SHOOT, TURN_1, ALIGN_1, DRIVE_2, ALIGN_2, PUSH,
    STRAFE, TURN_2, DRIVE_3, END
}

enum Beacon {
    ONE, TwO,
}

