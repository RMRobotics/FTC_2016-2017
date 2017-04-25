package org.firstinspires.ftc.rmrobotics.opmodes.feRMilab.util;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.rmrobotics.core.FeRMiLinear;
import org.firstinspires.ftc.rmrobotics.util.enums.Color;
import org.firstinspires.ftc.rmrobotics.util.enums.Direction;

/**
 * Created by Simon on 4/23/17.
 */

@TeleOp(name = "beaconPusher")
public class beaconPusher extends FeRMiLinear {
    @Override
    public void runOpMode() throws InterruptedException {
        super.initialize(Color.RED, DcMotor.RunMode.RUN_USING_ENCODER, Direction.FORWARD);

        double lP;
        double rP;
        while (opModeIsActive()) {
            telemetry.addData("LEFT:", colorLeftReader.read(0x04, 1)[0]);
            telemetry.addData("RIGHT:", colorRightReader.read(0x04, 1)[0]);
            telemetry.addData("RANGE:", rangeReader.read(0x04, 2)[0] + " " + rangeReader.read(0x04, 2)[1]);
            if (gamepad1.dpad_up) {
                lP = 1;
            } else if (gamepad1.dpad_down) {
                lP = -1;
            } else {
                lP = 0;
            }
            if (gamepad1.y) {
                rP = 1;
            } else if (gamepad1.a) {
                rP = -1;
            } else {
                rP = 0;
            }
            pushLeft.setPower(lP);
            pushRight.setPower(rP);
            telemetry.update();
        }
    }
}
