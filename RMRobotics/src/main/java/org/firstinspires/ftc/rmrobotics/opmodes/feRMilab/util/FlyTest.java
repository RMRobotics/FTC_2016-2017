package org.firstinspires.ftc.rmrobotics.opmodes.feRMilab.util;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.rmrobotics.core.FeRMiLinear;

/**
 * Created by Simon on 4/17/17.
 */

@TeleOp(name = "FlyTest")
public class FlyTest extends FeRMiLinear {

    @Override
    public void runOpMode() throws InterruptedException {
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
