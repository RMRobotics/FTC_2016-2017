package org.firstinspires.ftc.rmrobotics.opmodes.feRMilab.util;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by Simon on 1/11/2017.
 */

@TeleOp(name = "servoTest")
@Disabled
public class servoTest extends OpMode {

    private Servo servo;
    private double servoValue = 0.5;

    @Override
    public void init() {
        servo = hardwareMap.servo.get("liftHold");
    }

    @Override
    public void loop() {
        //turns beacon pusher servo;+
        if (gamepad1.x) {
            servoValue = 0.93;
        } else if (gamepad1.b) {
            servoValue = 0;
        }
        servo.setPosition(servoValue);
        telemetry.addData("pos", servo.getPosition());
    }
}
