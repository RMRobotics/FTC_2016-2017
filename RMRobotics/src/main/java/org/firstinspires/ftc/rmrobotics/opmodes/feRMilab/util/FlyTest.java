package org.firstinspires.ftc.rmrobotics.opmodes.feRMilab.util;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

/**
 * Created by Simon on 4/17/17.
 */

@TeleOp(name = "FlyTest")
public class FlyTest extends LinearOpMode {

    DcMotor flyL;
    DcMotor flyR;

    @Override
    public void runOpMode() {
        flyL = hardwareMap.dcMotor.get("flyL");
        flyR = hardwareMap.dcMotor.get("flyR");
        flyL.setDirection(DcMotorSimple.Direction.REVERSE);
        flyR.setDirection(DcMotorSimple.Direction.REVERSE);
        flyL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flyR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

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
            flyL.setPower(power);
            flyR.setPower(power);
            telemetry.addData("Power:", power);
            telemetry.update();
        }
    }
}
