package org.firstinspires.ftc.ftc5421.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.ftc5421.control.Axis;
import org.firstinspires.ftc.ftc5421.control.Controller;
import org.firstinspires.ftc.ftc5421.control.Joystick;
import org.firstinspires.ftc.ftc5421.core.RTeleOp;
import org.firstinspires.ftc.ftc5421.hardware.motor;

/**
 * Created by Simon on 9/22/2016.
 */

@TeleOp(name="Tank", group="5421")  // @Autonomous(...) is the other common choice
@Disabled
public class Tank extends RTeleOp{
    ElapsedTime runtime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

    motor FL;
    motor FR;
    motor BL;
    motor BR;

    @Override
    public void init_loop() {
        addTelemetry();
    }

    @Override
    public void start() {
        FL = motorMap.get("FL");
        FR = motorMap.get("FR");
        BL = motorMap.get("BL");
        BR = motorMap.get("BR");
        runtime.reset();
    }

    @Override
    protected void calculate() {
        double left = control.joystickValue(Controller.ONE, Joystick.LEFT, Axis.Y);
        double right = control.joystickValue(Controller.ONE, Joystick.RIGHT, Axis.Y);


        FL.setPower(left);
        FR.setPower(right);
        BL.setPower(left);
        BR.setPower(right);

        addTelemetry();
    }

    @Override
    protected String setConfigurationPath() {
        return "TheRMite.json";
    }

    private void addTelemetry() {
        telemetry.addData("TIME", runtime.time());
        telemetry.addData("FL", FL.getPower());
        telemetry.addData("FR", FR.getPower());
        telemetry.addData("BL", BL.getPower());
        telemetry.addData("BR", BR.getPower());
    }
}
