package org.firstinspires.ftc.rmrobotics.opmodes.feRMilab.autored;

import com.kauailabs.navx.ftc.AHRS;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Created by Simon on 2/12/17.
 */
@Autonomous(name = "RED: Shoot Ramp")
public class ShootRamp extends LinearOpMode{
    private ElapsedTime runtime = new ElapsedTime();

    private DcMotor FL;
    private DcMotor FR;
    private DcMotor BL;
    private DcMotor BR;

    private AHRS navx;

    private DeviceInterfaceModule dim;
    double startPos;
    protected DcMotor flyL;
    protected DcMotor flyR;
    protected DcMotor belt;
    protected Servo index;
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

        driveRobot(1500, -0.4);

        flyL.setPower(0.985);
        flyR.setPower(0.985);
        sleep(200);
        index.setPosition(0.5);
        sleep(200);
        belt.setPower(1.0);
        sleep(3000);

        setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //turn at an angle
        turnRobotCorner(45);

        //drive forward
        driveRobot(895, -.4);

        //face ramp
        turnRobot(130);

        //drive onto ramp
        driveRobot(2500, -.4);

        while (opModeIsActive()){
            addTelemetry();
        }
    }

    private void driveRobot(int distance, double power){
        startPos = FL.getCurrentPosition();
        while (FL.getCurrentPosition() - startPos > (-1*distance) && opModeIsActive()){
            setDrive(power);
        }
        setDrive(0);
        sleep(100);
    }

    private void turnRobotCorner(int a){
        while (Math.abs(navx.getYaw() + a) > 2 && opModeIsActive()) {
            int scale;
            if (navx.getYaw() + a > 0) {
                scale = -1;
            } else {
                scale = 1;
            }
            if (Math.abs(navx.getYaw()) < (a - 10)){
                setDrive(scale * 0.25, 0);
            } else {
                setDrive(scale * 0.07, 0);
            }
        }
        setDrive(0);
        sleep(100);
    }

    private void turnRobot(int a){
        while (Math.abs(navx.getYaw() + a) > 2 && opModeIsActive()) {
            int scale;
            if (navx.getYaw() + a > 0) {
                scale = -1;
            } else {
                scale = 1;
            }
            if (Math.abs(navx.getYaw()) < (a - 10)) {
                setDrive(scale * 0.25, scale*-0.25);
            } else {
                setDrive(scale * 0.07, scale*-0.07);
            }
        }
        setDrive(0);
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
}



