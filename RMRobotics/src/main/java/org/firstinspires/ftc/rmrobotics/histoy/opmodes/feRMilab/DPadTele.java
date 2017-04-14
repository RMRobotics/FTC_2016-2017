package org.firstinspires.ftc.rmrobotics.histoy.opmodes.feRMilab;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

/**
 * Created by RM Robotics on 1/13/2017.
 */

@TeleOp(name = "dPadTele")
@Disabled
public class DPadTele extends OpMode {

    private DcMotor FL;
    private DcMotor FR;
    private DcMotor BL;
    private DcMotor BR;

    @Override
    public void init() {
        FL = hardwareMap.dcMotor.get("FL");
        FL.setDirection(DcMotorSimple.Direction.REVERSE);
        FR = hardwareMap.dcMotor.get("FR");
        BL = hardwareMap.dcMotor.get("BL");
        BL.setDirection(DcMotorSimple.Direction.REVERSE);
        BR = hardwareMap.dcMotor.get("BR");
        FL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    @Override
    public void loop() {
        double forward = gamepad1.left_stick_y;
        double strafe = gamepad1.left_stick_x;
        double rotate = gamepad1.right_stick_x;
        double max = 0.7;

        List l = new ArrayList<>();
        l.add(Math.abs(forward + strafe + rotate));
        l.add(Math.abs(forward - strafe - rotate));
        l.add(Math.abs(forward - strafe + rotate));
        l.add(Math.abs(forward + strafe - rotate));
        if ((double) Collections.max(l) > max) {
            max = (double) Collections.max(l);
        }

        if (gamepad1.dpad_up){
            FL.setPower(.4);
            FR.setPower(.4);
            BL.setPower(.4);
            BR.setPower(.4);
        }
        else if (gamepad1.dpad_down){
            FL.setPower(-.4);
            FR.setPower(-.4);
            BL.setPower(-.4);
            BR.setPower(-.4);
        }
        else if (gamepad1.dpad_left){
            FL.setPower(-.4);
            FR.setPower(-.4);
            BL.setPower(.4);
            BR.setPower(.4);
        }
        else if (gamepad1.dpad_right){
            FL.setPower(.4);
            FR.setPower(.4);
            BL.setPower(-.4);
            BR.setPower(-.4);
        }
        else {
            FL.setPower((forward - strafe - rotate) / max);
            FR.setPower((forward + strafe + rotate) / max);
            BL.setPower((forward + strafe - rotate) / max);
            BR.setPower((forward - strafe + rotate)/max);
        }


    }
}
