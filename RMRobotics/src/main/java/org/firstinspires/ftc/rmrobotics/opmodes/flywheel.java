package org.firstinspires.ftc.rmrobotics.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by Simon on 11/10/16.
 */

@TeleOp(name="flywheel", group="5421")
public class flywheel extends OpMode {
    DcMotor m1;
    DcMotor m2;

    public void init() {
        m1 = hardwareMap.dcMotor.get("1");
        m2 = hardwareMap.dcMotor.get("2");
        m1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        m2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    @Override
    public void loop() {
        double power = gamepad1.left_stick_y;
        m1.setPower(power);
        m2.setPower(power);
    }
}
