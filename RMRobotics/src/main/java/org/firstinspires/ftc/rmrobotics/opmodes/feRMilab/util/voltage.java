package org.firstinspires.ftc.rmrobotics.opmodes.feRMilab.util;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;

/**
 * Created by Simon on 3/17/17.
 */

@Autonomous(name = "voltage")
public class voltage extends OpMode {

    private DcMotor belt;
    private DcMotor flyL;
    private DcMotor flyR;

    private Servo index;

    private VoltageSensor mc;

    @Override
    public void init() {
        belt = hardwareMap.dcMotor.get("belt");
        flyL = hardwareMap.dcMotor.get("flyL");
        flyL.setDirection(DcMotorSimple.Direction.REVERSE);
        flyR = hardwareMap.dcMotor.get("flyR");
        flyR.setDirection(DcMotorSimple.Direction.REVERSE);
        index = hardwareMap.servo.get("indexer");
        mc = hardwareMap.voltageSensor.get("Flywheel Controller 1");
    }

    @Override
    public void loop() {
        telemetry.addData("voltage", mc.getVoltage());
        double yee = mc.getVoltage()*-0.1242 + 2.421;
        telemetry.addData("power", yee);
        index.setPosition(0.5);
        belt.setPower(-1.0);
        flyL.setPower(yee);
        flyR.setPower(yee);
    }
}
