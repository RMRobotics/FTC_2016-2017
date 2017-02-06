package org.firstinspires.ftc.rmrobotics.opmodes.feRMilab;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

/**
 * Created by michaelblob on 12/4/16.
 */

@TeleOp(name = "feRMi - TELEOP", group = "feRMi")
public class FeRMiTele extends OpMode {

    private DcMotor FL;
    private DcMotor FR;
    private DcMotor BL;
    private DcMotor BR;
    private DcMotor flyL;
    private DcMotor flyR;
    private DcMotor belt;
    private DcMotor lift;

    private Servo harvester;
    private Servo beaconArm;
    private Servo index;
    private Servo liftHold;

    @Override
    public void init() {
        FL = hardwareMap.dcMotor.get("FL");
        FL.setDirection(DcMotorSimple.Direction.REVERSE);
        FR = hardwareMap.dcMotor.get("FR");
        BL = hardwareMap.dcMotor.get("BL");
        BL.setDirection(DcMotorSimple.Direction.REVERSE);
        BR = hardwareMap.dcMotor.get("BR");
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

        harvester = hardwareMap.servo.get("h");
        harvester.setPosition(0.5);
        beaconArm = hardwareMap.servo.get("swingArm");
        beaconArm.setPosition(0.5);
        index = hardwareMap.servo.get("indexer");
        index.setPosition(0.1);
        liftHold = hardwareMap.servo.get("liftHold");

        FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

    }

    @Override
    public void loop() {
        double max = 1.0;
        double forward = gamepad1.left_stick_y*0.8;
        double strafe = gamepad1.left_stick_x*0.8;
        double rotate = gamepad1.right_stick_x*0.7;
        telemetry.addData("FL: forward: ", forward + "+ strafe: " + strafe + "+ rotate: " + rotate);
        telemetry.addData("FR: forward: ", forward + "- strafe: " + strafe + "- rotate: " + rotate);
        telemetry.addData("BL: forward: ", forward + "- strafe: " + strafe + "+ rotate: " + rotate);
        telemetry.addData("BR: forward: ", forward + "+ strafe: " + strafe + "- rotate: " + rotate);

        List l = new ArrayList<>();
        l.add(Math.abs(forward + strafe + rotate));
        l.add(Math.abs(forward - strafe - rotate));
        l.add(Math.abs(forward - strafe + rotate));
        l.add(Math.abs(forward + strafe - rotate));
        if ((double) Collections.max(l) > max) {
            max = (double) Collections.max(l);
        }

        /*double driveWeight = 1.0;
        double turnWeight = 1.0;
        double res = 0.4;
        double[] voltages = DriveUtil.mecanumDrive(strafe, forward, driveWeight, rotate, turnWeight, res, resTog);*/

        FL.setPower((forward - strafe - rotate)/max);
        FR.setPower((forward + strafe + rotate)/max);
        BL.setPower((forward + strafe - rotate)/max);
        BR.setPower((forward - strafe + rotate)/max);

        boolean harvest = gamepad1.right_bumper;
        boolean eject = gamepad1.left_bumper;
        if (harvest && eject) {
            harvester.setPosition(0.5);
        } else if (harvest) {
            harvester.setDirection(Servo.Direction.FORWARD);
            harvester.setPosition(0);
        } else if (eject) {
            harvester.setDirection(Servo.Direction.REVERSE);
            harvester.setPosition(0);
        } else {
            harvester.setPosition(0.5);
        }

        boolean beltUp = gamepad1.dpad_up;
        boolean beltDown = gamepad1.dpad_down;
        if (beltUp && beltDown) {
            belt.setPower(0);
        } else if (beltUp) {
            belt.setPower(0.6);
        } else if (beltDown) {
            belt.setPower(-0.6);
        } else {
            belt.setPower(0);
        }
        //addTelemetry();

        if (gamepad2.y) {
            flyL.setPower(1);
            flyR.setPower(1);
        } else if (gamepad2.a) {
            flyL.setPower(-1);
            flyR.setPower(-1);
        } else {
            flyL.setPower(0);
            flyR.setPower(0);
        }

        if(gamepad2.x){
            beaconArm.setPosition(.68);
        } else if (gamepad2.b){
            beaconArm.setPosition(.20);
        }

        if (gamepad2.left_bumper){
            index.setPosition(.5);
        } else {
            index.setPosition(.1);
        }

        if (gamepad2.left_trigger > 0.3) {
            lift.setPower(-gamepad2.left_trigger/2);
        } else if (gamepad2.right_trigger > 0.3) {
            lift.setPower(gamepad2.right_trigger);
        } else {
            lift.setPower(0);
        }

        if (gamepad2.dpad_down) {
            liftHold.setPosition(1);
        } else {
            liftHold.setPosition(0.29);
        }

        addTelemetry();

    }
    private void addTelemetry() {
        telemetry.addData("1 Motor", FL.getPower() + " " + FR.getPower() + " " + BL.getPower() + " " + BR.getPower());
        telemetry.addData("2 Encoder", FL.getCurrentPosition() + " " + FR.getCurrentPosition() + " " + BL.getCurrentPosition() + " " + BR.getCurrentPosition());
        telemetry.update();
    }
}
