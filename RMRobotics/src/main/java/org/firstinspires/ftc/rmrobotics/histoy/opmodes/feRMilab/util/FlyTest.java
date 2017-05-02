package org.firstinspires.ftc.rmrobotics.histoy.opmodes.feRMilab.util;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

/**
 * Created by Simon on 4/17/17.
 */

@TeleOp(name = "FlyTest")
@Disabled
public class FlyTest extends LinearOpMode {

    DcMotor flyL;
    DcMotor flyR;

    DcMotor belt;

    @Override
    public void runOpMode() {
        flyL = hardwareMap.dcMotor.get("flyL");
        flyR = hardwareMap.dcMotor.get("flyR");
        flyL.setDirection(DcMotorSimple.Direction.REVERSE);
        flyR.setDirection(DcMotorSimple.Direction.REVERSE);
        flyL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flyR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        belt = hardwareMap.dcMotor.get("belt");
        belt.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();

        double power = 0;
        while (opModeIsActive()) {
            if (power > 0.7) {
                power = 0.7;
            } else if (power < 0) {
                power = 0;
            }
            if (gamepad1.y) {
                power += 0.001;
            } else if (gamepad1.a) {
                power -= 0.001;
            }
            if (gamepad1.dpad_up) {
                belt.setPower(1.0);
            } else if (gamepad1.dpad_down) {
                belt.setPower(-1.0);
            } else {
                belt.setPower(0);
            }
            if (gamepad1.right_bumper) {
                flyL.setPower(power);
                flyR.setPower(power);
            } else {
                flyL.setPower(0);
                flyR.setPower(0);
            }
            telemetry.addData("Power:", power);
            telemetry.update();
        }
    }
}
