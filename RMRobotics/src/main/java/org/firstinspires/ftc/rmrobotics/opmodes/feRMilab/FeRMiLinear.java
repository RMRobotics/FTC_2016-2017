package org.firstinspires.ftc.rmrobotics.opmodes.feRMilab;

import com.kauailabs.navx.ftc.AHRS;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchImpl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.rmrobotics.util.Color;
import org.firstinspires.ftc.rmrobotics.util.Direction;

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
    protected Servo harvester;
    protected Servo index;
    protected Servo liftHold;

    protected AHRS navx;

    protected I2cDevice colorBack;
    protected I2cDeviceSynch colorBackReader;
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
        harvester = hardwareMap.servo.get("h");
        index = hardwareMap.servo.get("indexer");
        liftHold = hardwareMap.servo.get("liftHold");

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

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        runtime.reset(); // reset runtime counter
        navx.zeroYaw(); // reset navx yaw value

        // initialize servo positions
        swingArm.setPosition(0.5);
        harvester.setPosition(0.5);
        index.setPosition(0.1);
        liftHold.setPosition(0.18);
    }

    protected void driveTime(double power, int time){
        //drives at power for time
        initTime = runtime.milliseconds();
        while (runtime.milliseconds() - initTime < time && opModeIsActive()) {
            setDrive(0.4);
        }
        setDrive(0);
    }

    protected void driveEncoder(double power, int encoder){

    }

    protected void driveToRange(double power, int range){
        while (rangeReader.read(0x04, 2)[0] > range && opModeIsActive()) {
            setDrive(power);
        }
        setDrive(0);
    }

    protected void driveAwayRange(double power, int range){
        while (rangeReader.read(0x04, 2)[0] < range && opModeIsActive()) {
            setDrive(power);
        }
        setDrive(0);
    }

    protected void turnCenter(int degree, double power){

        // while robot is more than 2 degrees away from target degree
        while (Math.abs(navx.getYaw() - degree) > 2 && opModeIsActive()) {
            int correct = scale;
            if (navx.getYaw() - degree <= 0) {
                // if robot is turning in the right direction
                correct = scale;
            } else {
                // if robot has overturned
                correct = scale * -1;
            }
            if (Math.abs(navx.getYaw() - degree) > 15){
                // if robot is more than 15 degrees away from target degree
                // faster speed
                setDrive(correct * power, -correct * power);
            } else {
                // if robot is within 15 degrees away from target degree
                // slower speed
                setDrive(correct * (power/5.5), -correct * (power/5.5));
            }
        }
        setDrive(0);
        sleep(100);
    }

    protected void turnCorner(int degree, double power, Direction tDir){

        // finds the difference between the target and the starting angle
        float delta = degree - navx.getYaw();
        // sets the magnitude of the turn (absolute value of delta)
        float mag = Math.abs(delta);
        // whether or not you are turning left or right
        float dir = Math.signum(delta);
        // while robot is more than 2 degrees away from the target angle
        while(mag > 2 && opModeIsActive()){
            // if the left side of the drive train is used
            if(tDir == Direction.LEFT){
                setDrive(dir*power, 0);
            }
            // if the right side is used
            else if(tDir == Direction.RIGHT){
                setDrive(0, -dir*power);
            }
            // update distance from target angle
            delta = degree - navx.getYaw();
            mag = Math.abs(delta);
            dir = Math.signum(delta);
        }

/*        while (Math.abs(navx.getYaw() - degree) > 2 && opModeIsActive()) {
            int correct;
            if (navx.getYaw() - degree <= 0) {
                // if robot is turning in the right direction
                correct = scale;
            } else {
                //if robot has overturned
                correct = scale * -1;
            }
            if (Math.abs(navx.getYaw() - degree) > 15){
                // if robot is more than 15 degrees away from target degree
                //faster speed
                if (correct == 1){
                    setDrive(0, correct * power);
                }
                else if (correct == -1){
                    setDrive(correct * power, 0);
                }
            } else {
                // if robot is within 15 degrees away from target degree
                //slower speed
                if (correct == 1){
                    setDrive(0, correct * (0.07));
                }
                else if (correct == -1){
                    setDrive(correct * (power/0.07), 0);
                }
            }
        }*/
        setDrive(0);
        sleep(100);
    }

    protected void addTelemetry() {
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
