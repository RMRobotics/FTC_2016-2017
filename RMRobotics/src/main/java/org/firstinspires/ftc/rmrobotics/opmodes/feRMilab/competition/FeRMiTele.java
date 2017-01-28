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
    private DcMotor belt;

    private Servo harvester;
    private Servo index;
    private Servo beaconArm;

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
        belt = hardwareMap.dcMotor.get("belt");
        belt.setDirection(DcMotorSimple.Direction.REVERSE);
        FL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        harvester = hardwareMap.servo.get("h");
        beaconArm = hardwareMap.servo.get("swingArm");
        index = hardwareMap.servo.get("indexer");
    }

    @Override
    public void loop() {
        double max = 1.0;
        double forward = gamepad1.left_stick_y*0.8;
        double strafe = gamepad1.left_stick_x*0.8;
        double rotate = gamepad1.right_stick_x*0.7;
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
            belt.setPower(0);
        } else if (beltUp) {
            belt.setPower(0.6);
        } else if (beltDown) {
            belt.setPower(-0.6);
        } else {
            belt.setPower(0);
        }
        //belt.setPower(0.7);
//        harvester.setPosition(0);
//        harvester.setDirection(Servo.Direction.FORWARD);
        addTelemetry();

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

        if (gamepad2.left_bumper == true){
            index.setPosition(.5);
        }else if (gamepad2.right_bumper==true){
            index.setPosition(.1);
        }

    }
    private void addTelemetry() {
        telemetry.addData("1 Motor", FL.getPower() + " " + FR.getPower() + " " + BL.getPower() + " " + BR.getPower());
        telemetry.addData("2 Encoder", FL.getCurrentPosition() + " " + FR.getCurrentPosition() + " " + BL.getCurrentPosition() + " " + BR.getCurrentPosition());
        telemetry.update();
    }
}
