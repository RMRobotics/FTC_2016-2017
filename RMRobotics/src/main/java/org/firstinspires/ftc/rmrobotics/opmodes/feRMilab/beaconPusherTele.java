package org.firstinspires.ftc.rmrobotics.opmodes.feRMilab;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by RM Robotics on 1/11/2017.
 */

@TeleOp(name = "beaconPusherCalibration")
public class beaconPusherTele extends OpMode {

    private Servo beaconPusher;
    private int servoValue = 0;


    @Override
    public void init() {
        beaconPusher = hardwareMap.servo.get("swingArm");
    }

    @Override
    public void loop() {
        //turns beacon pusher servo;+
        if (gamepad1.x) {
            servoValue+=.1;
            beaconPusher.setPosition(servoValue);
        } else if (gamepad1.b) {
            servoValue -= .1;
            beaconPusher.setPosition(servoValue);
        }
    }
}
