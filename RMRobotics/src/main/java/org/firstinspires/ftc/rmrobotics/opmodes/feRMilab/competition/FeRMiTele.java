package org.firstinspires.ftc.rmrobotics.opmodes.feRMilab.competition;

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

    private Servo beaconL;
    private Servo beaconR;

    private Servo harvester;
    private Servo belt;
    private CRServo index;

    @Override
    public void init() {
        FL = hardwareMap.dcMotor.get("FL");
        FL.setDirection(DcMotorSimple.Direction.REVERSE);
        FR = hardwareMap.dcMotor.get("FR");
        BL = hardwareMap.dcMotor.get("BL");
        BL.setDirection(DcMotorSimple.Direction.REVERSE);
        BR = hardwareMap.dcMotor.get("BR");
        flyL = hardwareMap.dcMotor.get("flyL");
        flyR = hardwareMap.dcMotor.get("flyR");

        beaconL = hardwareMap.servo.get("beaconL");
        beaconR = hardwareMap.servo.get("beaconR");
        beaconR.setDirection(Servo.Direction.REVERSE);
        harvester = hardwareMap.servo.get("h");
        belt = hardwareMap.servo.get("b");
        index = hardwareMap.crservo.get("indexer");
    }

    @Override
    public void loop() {
        double forward = gamepad1.left_stick_y;
        double strafe = gamepad1.left_stick_x;
        double rotate = gamepad1.right_stick_x;
        double max = 1;
        List l = new ArrayList<>();
        l.add(Math.abs(forward + strafe + rotate));
        l.add(Math.abs(forward - strafe - rotate));
        l.add(Math.abs(forward - strafe + rotate));
        l.add(Math.abs(forward + strafe - rotate));
        if ((double) Collections.max(l) > 1) {
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
            harvester.setPosition(0);
            harvester.setDirection(Servo.Direction.FORWARD);
        } else if (eject) {
            harvester.setPosition(0);
            harvester.setDirection(Servo.Direction.REVERSE);
        } else {
            harvester.setPosition(0.5);
        }

        boolean beltUp = gamepad1.dpad_up;
        boolean beltDown = gamepad1.dpad_down;
        if (beltUp && beltDown) {
            belt.setPosition(0.5);
        } else if (beltUp) {
            belt.setPosition(0);
            belt.setDirection(Servo.Direction.REVERSE);
        } else if (beltDown) {
            belt.setPosition(0);
            belt.setDirection(Servo.Direction.FORWARD);
        } else {
            belt.setPosition(0.5);
        }

        if (gamepad2.left_bumper) {
            beaconL.setPosition(1);
        } else {
            beaconL.setPosition(0);
        }
        if (gamepad2.right_bumper) {
            beaconR.setPosition(0);
        } else {
            beaconR.setPosition(1);
        }

        if (gamepad2.a) {
            flyL.setPower(1.0);
            flyR.setPower(1.0);
        } else if (gamepad2.y) {
            flyL.setPower(-1);
            flyR.setPower(-1);
        } else {
            flyL.setPower(0);
            flyR.setPower(0);
        }

        if (gamepad2.b) {
            index.setPower(1.0);
        } else if (gamepad2.x) {
            index.setPower(-1);
        } else {
            index.setPower(0);
        }
    }
}
