package org.firstinspires.ftc.rmrobotics.opmodes.feRMilab.util;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by RM Robotics on 1/11/2017.
 */

@TeleOp(name = "beaconPusherCalibration")
@Disabled
public class beaconPusherTele extends OpMode {

    private Servo beaconPusher;
    private Servo h;
    private double servoValue = 0.5;

    @Override
    public void init() {
        beaconPusher = hardwareMap.servo.get("swingArm");
        h = hardwareMap.servo.get("h");
        h.setPosition(0.5);
    }

    @Override
    public void loop() {
        //turns beacon pusher servo;+
        if (gamepad1.x) {
            servoValue+=.01;
            beaconPusher.setPosition(servoValue);
        } else if (gamepad1.b) {
            servoValue -= .01;
            beaconPusher.setPosition(servoValue);
        }
        telemetry.addData("pos", beaconPusher.getPosition());
    }
}
