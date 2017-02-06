package org.firstinspires.ftc.rmrobotics.opmodes.feRMilab.util;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by RM Robotics on 1/11/2017.
 */

@TeleOp(name = "servoCalibration")
public class servoCalibration extends OpMode {

    private Servo servo;
    private Servo h;
    private double servoValue = 0.5;

    @Override
    public void init() {
        servo = hardwareMap.servo.get("liftHold");
        h = hardwareMap.servo.get("h");
        h.setPosition(0.5);
    }

    @Override
    public void loop() {
        //turns beacon pusher servo;+
        if (gamepad1.x) {
            servoValue+=.01;
            servo.setPosition(servoValue);
        } else if (gamepad1.b) {
            servoValue -= .01;
            servo.setPosition(servoValue);
        }
        telemetry.addData("pos", servo.getPosition());
    }
}
