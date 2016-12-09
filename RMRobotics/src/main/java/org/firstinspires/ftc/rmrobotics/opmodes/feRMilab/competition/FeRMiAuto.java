package org.firstinspires.ftc.rmrobotics.opmodes.feRMilab.competition;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by Simon on 12/6/16.
 */

@Autonomous(name = "feRMi - AUTONOMOUS", group = "feRMi")
public class FeRMiAuto extends LinearOpMode{

    private DcMotor FL;
    private DcMotor FR;
    private DcMotor BL;
    private DcMotor BR;
    private DcMotor flyL;
    private DcMotor flyR;
    private DcMotor belt;

    private Servo beaconL;
    private Servo beaconR;
    private Servo harvester;
    private Servo index;

    @Override
    public void runOpMode() throws InterruptedException {
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
        belt = hardwareMap.dcMotor.get("b");
        beaconL = hardwareMap.servo.get("beaconL");
        beaconR = hardwareMap.servo.get("beaconR");
        beaconR.setDirection(Servo.Direction.REVERSE);
        harvester = hardwareMap.servo.get("h");
        index = hardwareMap.servo.get("indexer");

        waitForStart();

        setEncoder(1120, 0.2);
        sleep(2000);

        flyL.setPower(1.0);
        flyR.setPower(1.0);
        belt.setPower(1.0);
        sleep(7500);


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

    private void setEncoder(int e, double p) {
        FL.setTargetPosition(e);
        FL.setPower(p);
        FR.setTargetPosition(e);
        FR.setPower(p);
        BL.setTargetPosition(e);
        BL.setPower(p);
        BR.setTargetPosition(e);
        BR.setPower(p);
    }

    private void setEncoder(int e1, int e2, double p1, double p2) {
        FL.setTargetPosition(e1);
        FL.setPower(p1);
        FR.setTargetPosition(e2);
        FR.setPower(p2);
        BL.setTargetPosition(e1);
        BL.setPower(p1);
        BR.setTargetPosition(e2);
        BR.setPower(p2);
    }

    private void setEncoder(int e1, int e2, int e3, int e4, double p1, double p2, double p3, double p4) {
        FL.setTargetPosition(e1);
        FL.setPower(p1);
        FR.setTargetPosition(e2);
        FR.setPower(p2);
        BL.setTargetPosition(e3);
        BL.setPower(p3);
        BR.setTargetPosition(e4);
        BR.setPower(p4);
    }
}
