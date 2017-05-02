package org.firstinspires.ftc.rmrobotics.opmodes.feRMilab.tele;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

/**
 * Created by Simon on 12/4/16.
 */
// V.JOSH

@TeleOp(name = "feRMi - CALIBRATION", group = "feRMi")
@Disabled
public class FeRMiCalibration extends OpMode {

    private DcMotor FL;
    private DcMotor FR;
    private DcMotor BL;
    private DcMotor BR;
    private DcMotor flyL;
    private DcMotor flyR;
    private DcMotor belt;
    private DcMotor lift;

    private Servo beaconArm;
    private Servo index;
    private Servo liftHold;

    private boolean triggered = false;
    private double power = 0;

    @Override
    public void init() {
        FL = hardwareMap.dcMotor.get("FL");
        FR = hardwareMap.dcMotor.get("FR");
        FR.setDirection(DcMotorSimple.Direction.REVERSE);
        BL = hardwareMap.dcMotor.get("BL");
        BR = hardwareMap.dcMotor.get("BR");
        BR.setDirection(DcMotorSimple.Direction.REVERSE);
        flyL = hardwareMap.dcMotor.get("flyL");
        flyL.setDirection(DcMotorSimple.Direction.REVERSE);
        flyR = hardwareMap.dcMotor.get("flyR");
        flyR.setDirection(DcMotorSimple.Direction.REVERSE);
        belt = hardwareMap.dcMotor.get("belt");
        belt.setDirection(DcMotorSimple.Direction.REVERSE);
        lift = hardwareMap.dcMotor.get("lift");
        lift.setDirection(DcMotorSimple.Direction.REVERSE);
        FL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        beaconArm = hardwareMap.servo.get("swingArm");
        beaconArm.setPosition(0.495);
        index = hardwareMap.servo.get("indexer");
        index.setPosition(0.12);
        liftHold = hardwareMap.servo.get("liftHold");
        liftHold.setPosition(0.4);

        FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

    }

    @Override
    public void loop() {
        double max = 1.0;
        double forward = -gamepad1.left_stick_y;
        double strafe = gamepad1.left_stick_x;
        double rotate = gamepad1.right_stick_x;

        List l = new ArrayList<>();
        l.add(Math.abs(forward + strafe + rotate));
        l.add(Math.abs(forward - strafe - rotate));
        l.add(Math.abs(forward - strafe + rotate));
        l.add(Math.abs(forward + strafe - rotate));
        if ((double) Collections.max(l) > max) {
            max = (double) Collections.max(l);
        }

        if (gamepad1.dpad_up) {
            setDrive(1, 1, 1, 1);
        } else if (gamepad1.dpad_down) {
            setDrive(-1, -1, -1, -1);
        } else if (gamepad1.dpad_left) {
            setDrive(-1, 1, 1, -1);
        } else if (gamepad1.dpad_right) {
            setDrive(1, -1, -1, 1);
        } else {
            FL.setPower((forward + strafe + rotate) / max);
            FR.setPower((forward - strafe - rotate) / max);
            BL.setPower((forward - strafe + rotate) / max);
            BR.setPower((forward + strafe - rotate) / max);
        }

        boolean harvest = gamepad1.right_bumper;
        boolean eject = gamepad1.left_bumper;
        if (harvest && eject) {
            belt.setPower(0);
        } else if (harvest) {
            belt.setPower(1.0);
        } else if (eject) {
            belt.setPower(-1.0);
        } else {
            belt.setPower(0);
        }

        if (gamepad2.y) {
            flyL.setPower(power);
            flyR.setPower(power);
        } else if (gamepad2.a) {
            flyL.setPower(-.985);
            flyR.setPower(-.985);
        } else if (gamepad2.right_bumper) {
            flyL.setPower(1);
            flyR.setPower(1);
        } else {
            flyL.setPower(0);
            flyR.setPower(0);
        }

        if (gamepad2.x) {
            power -= 0.0001;
        } else if (gamepad2.b) {
            power += 0.0001;
        }

        telemetry.addData("voltage", power);

//        if(gamepad2.x){
//            beaconArm.setPosition(1.0);
//        } else if (gamepad2.b){
//            beaconArm.setPosition(0);
//        } else {
//            beaconArm.setPosition(0.495);
//        }

        if (gamepad2.left_bumper){
            index.setPosition(.5);
        } else {
            index.setPosition(.12);
        }

//        if (gamepad2.left_trigger > 0.3) {
//            lift.setPower(-gamepad2.left_trigger);
//        } else if (gamepad2.right_trigger > 0.3) {
//            lift.setPower(gamepad2.right_trigger/2);
//        } else {
//            lift.setPower(0);
//        }

//        if (gamepad2.dpad_down) {
//            liftHold.setPosition(1);
//            triggered = true;
//        } else if (triggered) {
//            liftHold.setPosition(0);
//        } else {
//            liftHold.setPosition(0.4);
//        }
        liftHold.setPosition(0.4);

//        addTelemetry();

    }
    private void addTelemetry() {
        telemetry.addData("1 Motor", FL.getPower() + " " + FR.getPower() + " " + BL.getPower() + " " + BR.getPower());
        telemetry.addData("2 Encoder", FL.getCurrentPosition() + " " + FR.getCurrentPosition() + " " + BL.getCurrentPosition() + " " + BR.getCurrentPosition());
        telemetry.update();
    }

    protected void setDrive(double p1, double p2, double p3, double p4) {
        FL.setPower(p1);
        FR.setPower(p2);
        BL.setPower(p3);
        BR.setPower(p4);
    }
}
