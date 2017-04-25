package org.firstinspires.ftc.rmrobotics.core;

import com.kauailabs.navx.ftc.AHRS;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchImpl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.rmrobotics.util.enums.Color;
import org.firstinspires.ftc.rmrobotics.util.enums.Direction;
import org.firstinspires.ftc.rmrobotics.util.enums.Drive;

/**
 * Created by Simon on 2/6/17.
 */

public abstract class FeRMiLinear extends LinearOpMode {
    protected ElapsedTime runtime = new ElapsedTime();

    protected DcMotor FL;
    protected DcMotor FR;
    protected DcMotor BL;
    protected DcMotor BR;
    protected DcMotor flyL;
    protected DcMotor flyR;
    protected DcMotor belt;

    protected Servo swingArm;
    protected Servo index;
    protected Servo liftHold;

    protected CRServo pushLeft;
    protected CRServo pushRight;

    protected AHRS navx;

    protected VoltageSensor flyMC;

    protected I2cDevice colorCenter;
    protected I2cDeviceSynch colorCenterReader;
    protected I2cDevice colorRight;
    protected I2cDeviceSynch colorRightReader;
    protected I2cDevice colorLeft;
    protected I2cDeviceSynch colorLeftReader;

    protected DeviceInterfaceModule dim;

    protected I2cDevice range;
    protected I2cDeviceSynch rangeReader;

    protected int scale;
    protected double initTime;
    protected double voltage;

    protected Color left;
    protected Color right;
    protected Direction beacon;

    public void initialize(Color c, DcMotor.RunMode r, Direction direction) {
        // motor initialization
        FL = hardwareMap.dcMotor.get("FL");
        FR = hardwareMap.dcMotor.get("FR");
        BL = hardwareMap.dcMotor.get("BL");
        BR = hardwareMap.dcMotor.get("BR");
        FR.setDirection(DcMotor.Direction.REVERSE);
        BR.setDirection(DcMotor.Direction.REVERSE);
        setZeroMode(DcMotor.ZeroPowerBehavior.BRAKE);
        setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setMode(r);
        flyL = hardwareMap.dcMotor.get("flyL");
        flyR = hardwareMap.dcMotor.get("flyR");
        flyL.setDirection(DcMotorSimple.Direction.REVERSE);
        flyR.setDirection(DcMotorSimple.Direction.REVERSE);
        belt = hardwareMap.dcMotor.get("belt");
        belt.setDirection(DcMotorSimple.Direction.REVERSE);

        // servo initialization
        swingArm = hardwareMap.servo.get("swingArm");
        index = hardwareMap.servo.get("indexer");
        liftHold = hardwareMap.servo.get("liftHold");

        // crservo initialization
        pushLeft = hardwareMap.crservo.get("pL");
        pushLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        pushRight = hardwareMap.crservo.get("pR");
        pushLeft.setPower(0);
        pushRight.setPower(0);

        // navx initialization and calibration
        dim = hardwareMap.deviceInterfaceModule.get("dim");
        navx = AHRS.getInstance(dim, 0, AHRS.DeviceDataType.kProcessedData, (byte) 50);
        while (navx.isCalibrating()) {
            telemetry.addData("Status", !navx.isCalibrating());
            telemetry.update();
        }

        // voltage sensor initialization
        flyMC = hardwareMap.voltageSensor.get("Flywheel Controller 1");

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
        switch (c) {
            case RED:
                dim.setLED(0, false); // blue
                dim.setLED(1, true); // red
                break;
            case BLUE:
                dim.setLED(1, false);
                dim.setLED(0, true);
                break;
            case NEITHER:
                dim.setLED(1, false);
                dim.setLED(1, true);
                break;
            default:
                dim.setLED(0, false);
                dim.setLED(0, true);
                break;
        }

        switch (direction){
            case FORWARD:
                scale = 1;
                break;
            case BACKWARD:
                scale = -1;
                break;
            default:
                scale = -1;
                break;
        }

        left = Color.NEITHER;
        right = Color.NEITHER;
        beacon = Direction.NONE;

        voltage = flyMC.getVoltage();

        telemetry.addData("Voltage", voltage);
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        runtime.reset(); // reset runtime counter
        navx.zeroYaw(); // reset navx yaw value

        // initialize servo positions
        swingArm.setPosition(0.495);
        index.setPosition(0.1);
        liftHold.setPosition(0);
    }

    protected void driveStop(Drive type, int val, double power) {
        drive(type, val, power);
        setDrive(0);
    }

    protected void drive(Drive type, int val, double power) {
        switch (type) {
            case TIME:
                initTime = runtime.milliseconds();
                while (runtime.milliseconds() - initTime < val && opModeIsActive()) {
                    setDrive(power);
                }
//                setDrive(0);
                break;
            case ENCODER:
                double mag = Math.abs(power);
                val = val*scale;
                double dir = Math.signum(val - FL.getCurrentPosition());
                setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//                setEnc(val);
                int shift = 0;
                // TODO: check to see if acceleration code functions properly
                while (Math.abs(FL.getCurrentPosition() - val) > 5 && opModeIsActive()) {
                    telemetry.addData("current Encoder value: ", FL.getCurrentPosition());
                    telemetry.update();
                    if (shift * 0.02 < mag) {
                        setDrive(scale * dir * shift * 0.05);
                        shift ++;
                        sleep(200);
                    } else {
                        setDrive(scale * dir * mag);
                    }
                }
//                setDrive(0);
                break;
            case RANGE:
                float delta = val - rangeReader.read(0x04, 2)[0];
                dir = Math.signum(delta);
                if (dir > 0) {
                    while (rangeReader.read(0x04, 2)[0] < val && opModeIsActive()) {
                        setDrive(-scale * power);
                    }
                } else if (dir < 0) {
                    while (rangeReader.read(0x04, 2)[0] > val && opModeIsActive()) {
                        setDrive(scale*power);
                    }
                }
//                setDrive(0);
                break;
            default:
                break;
        }
    }

    protected void turn(Direction side, int degree, double power) {
        // finds the difference between the target and the starting angle
        float delta = degree - navx.getYaw();

        // sets the magnitude of the turn (absolute value of delta)
        float mag = Math.abs(delta);

        // whether or not you are turning left or right
        float dir = Math.signum(delta);

        // while robot is more than 2 degrees away from the target angle
        while(mag > 2 && opModeIsActive()){
            if(mag < 12){
                power = 0.07;
            }
            switch (side) {
                case CENTER:
                    setDrive(dir*power, -dir*power);
                    break;
                case LEFT:
                    setDrive(dir*power, 0);
                    break;
                case RIGHT:
                    setDrive(0, -dir*power);
                    break;
                default:
                    setDrive(0);
                    break;
            }

            // update distance from target angle
            delta = degree - navx.getYaw();
            mag = Math.abs(delta);
            dir = Math.signum(delta);
        }
        setDrive(0);
    }

    protected void addTelemetry() {
        telemetry.addData("1 Time", runtime.seconds());
        telemetry.addData("2 Yaw", navx.getYaw());
        telemetry.addData("4 ColorCenter", colorCenterReader.read(0x08, 1)[0] & 0xFF);
        telemetry.addData("5 ColorLeft", colorLeftReader.read(0x04, 1)[0] & 0xFF);
        telemetry.addData("6 ColorRight", colorRightReader.read(0x04, 1)[0] & 0xFF);
        telemetry.addData("7 Range", rangeReader.read(0x04, 2)[0] + " " + rangeReader.read(0x04, 2)[1]);
        telemetry.addData("8 Motor", FL.getPower() + " " + FR.getPower() + " " + BL.getPower() + " " + BR.getPower());
        telemetry.addData("9 Encoder", FL.getCurrentPosition() + " " + FR.getCurrentPosition() + " " + BL.getCurrentPosition() + " " + BR.getCurrentPosition());
        telemetry.update();
    }

    protected void setMode(DcMotor.RunMode r) {
        FL.setMode(r);
        FR.setMode(r);
        BL.setMode(r);
        BR.setMode(r);
    }

    protected void setZeroMode(DcMotor.ZeroPowerBehavior z) {
        FL.setZeroPowerBehavior(z);
        FR.setZeroPowerBehavior(z);
        BL.setZeroPowerBehavior(z);
        BR.setZeroPowerBehavior(z);
    }

    protected void setDrive(double p) {
        FL.setPower(p);
        FR.setPower(p);
        BL.setPower(p);
        BR.setPower(p);
    }

    protected void setDrive(double p1, double p2) {
        FL.setPower(p1);
        FR.setPower(p2);
        BL.setPower(p1);
        BR.setPower(p2);
    }

    protected void setDrive(double p1, double p2, double p3, double p4) {
        FL.setPower(p1);
        FR.setPower(p2);
        BL.setPower(p3);
        BR.setPower(p4);
    }

    protected void setEnc(int p) {
        FL.setTargetPosition(FL.getCurrentPosition() + p);
        FR.setTargetPosition(FR.getCurrentPosition() + p);
        BL.setTargetPosition(BL.getCurrentPosition() + p);
        BR.setTargetPosition(BR.getCurrentPosition() + p);
    }

    protected void setEnc(int p1, int p2, int p3, int p4) {
        FL.setTargetPosition(FL.getCurrentPosition() + p1);
        FR.setTargetPosition(FR.getCurrentPosition() + p2);
        BL.setTargetPosition(BL.getCurrentPosition() + p3);
        BR.setTargetPosition(BR.getCurrentPosition() + p4);
    }

}
