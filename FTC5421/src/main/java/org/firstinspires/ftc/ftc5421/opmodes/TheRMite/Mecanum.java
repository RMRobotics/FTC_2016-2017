package org.firstinspires.ftc.ftc5421.opmodes.TheRMite;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.ftc5421.control.Axis;
import org.firstinspires.ftc.ftc5421.control.Controller;
import org.firstinspires.ftc.ftc5421.control.Joystick;
import org.firstinspires.ftc.ftc5421.core.RTeleOp;
import org.firstinspires.ftc.ftc5421.hardware.motor;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

@TeleOp(name="Mecanum", group="TheRMite")  // @Autonomous(...) is the other common choice
@Disabled
public class Mecanum extends RTeleOp {
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
        double forward = control.joystickValue(Controller.ONE, Joystick.LEFT, Axis.Y);
        double strafe = control.joystickValue(Controller.ONE, Joystick.LEFT, Axis.X);
        double rotate = control.joystickValue(Controller.ONE, Joystick.RIGHT, Axis.X);
        double max = maxCheck(forward, strafe, rotate);

        FL.setPower((forward + strafe + rotate)/max);
        FR.setPower((forward - strafe - rotate)/max);
        BL.setPower((forward - strafe + rotate)/max);
        BR.setPower((forward + strafe - rotate)/max);

        addTelemetry();
    }

    @Override
    protected String setConfigurationPath() {
        return "TheRMite.json";
    }

    @Override
    public void stop() {

    }

    private double maxCheck(double f, double s, double r) {
        List l = new ArrayList();
        l.add(Math.abs(f + s + r));
        l.add(Math.abs(f - s - r));
        l.add(Math.abs(f - s + r));
        l.add(Math.abs(f + s - r));
        if ((double) Collections.max(l) > 1) {
            return (double) Collections.max(l);
        } else {
            return 1;
        }
    }

    private void addTelemetry() {
        telemetry.addData("TIME", runtime.time());
        telemetry.addData("FL", FL.getPower());
        telemetry.addData("FR", FR.getPower());
        telemetry.addData("BL", BL.getPower());
        telemetry.addData("BR", BR.getPower());
    }
}
