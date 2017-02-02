package org.firstinspires.ftc.rmrobotics.opmodes.feRMilab;

import com.kauailabs.navx.ftc.AHRS;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Created by RM Robotics on 2/2/2017.
 */
@Autonomous(name = "ramp")
public class rampAutoLinear extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();

    private DcMotor FL;
    private DcMotor FR;
    private DcMotor BL;
    private DcMotor BR;

    private AHRS navx;

    private DeviceInterfaceModule dim;

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
        setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        dim = hardwareMap.deviceInterfaceModule.get("dim");

        navx = AHRS.getInstance(dim, 0, AHRS.DeviceDataType.kProcessedData, (byte) 50);

        while (navx.isCalibrating()) {
            telemetry.addData("Status", !navx.isCalibrating());
            telemetry.update();
        }

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        dim.setLED(1, true);

        navx.zeroYaw();

        waitForStart();
        runtime.reset(); // reset runtime counter

        //turn at an angle
        setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        while (Math.abs(navx.getYaw() + 42) > 2 && opModeIsActive()) {
            int scale;
            if (navx.getYaw() + 42 > 0) {
                scale = -1;
            } else {
                scale = 1;
            }
            if (Math.abs(navx.getYaw()) < 30) {
                setDrive(scale * 0.25, 0);
            } else {
                setDrive(scale * 0.07, 0);
            }
        }
//        addTelemetry();

        setDrive(0);
        sleep(100);

        //drive forward
        double startPos = FL.getCurrentPosition();
        while (FL.getCurrentPosition() - startPos > -1000 && opModeIsActive()){
            setDrive(-.4);
        }
//        addTelemetry();

        setDrive(0, 0, 0, 0);
        sleep(100);

        //turn at an angle
        setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        while (Math.abs(navx.getYaw()) < 140 && opModeIsActive()) {
            int scale;
            if (navx.getYaw() + 140 > 0) {
                scale = -1;
            } else {
                scale = 1;
            }
            if (Math.abs(navx.getYaw() + 140) > 20) {
                setDrive(scale * 0.25, scale*-0.25);
            } else {
                setDrive(scale * 0.07, scale*-0.07);
            }
        }
//        addTelemetry();

        setDrive(0);
        sleep(100);

        //drive forward
        startPos = FL.getCurrentPosition();
        while (FL.getCurrentPosition() - startPos > -500 && opModeIsActive()){
            setDrive(-.4);
        }
//        addTelemetry();

        setDrive(0, 0, 0, 0);
        sleep(100);
    }

    private void addTelemetry() {
        telemetry.addData("1 Time", runtime.seconds());
        telemetry.addData("2 Yaw", navx.getYaw());
//        telemetry.addData("3 ColorBack", colorBackReader.read(0x08, 1)[0] & 0xFF);
//        telemetry.addData("4 ColorCenter", colorCenterReader.read(0x08, 1)[0] & 0xFF);
//        telemetry.addData("5 ColorLeft", colorLeftReader.read(0x04, 1)[0] & 0xFF);
//        telemetry.addData("6 ColorRight", colorRightReader.read(0x04, 1)[0] & 0xFF);
        // telemetry.addData("7 Range", rangeReader.read(0x04, 2)[0] + " " + rangeReader.read(0x04, 2)[1]);
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


}
