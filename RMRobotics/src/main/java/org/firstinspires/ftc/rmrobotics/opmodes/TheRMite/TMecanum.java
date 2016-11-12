package org.firstinspires.ftc.rmrobotics.opmodes.TheRMite;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.rmrobotics.control.Axis;
import org.firstinspires.ftc.rmrobotics.control.Controller;
import org.firstinspires.ftc.rmrobotics.control.Joystick;
import org.firstinspires.ftc.rmrobotics.core.RTeleOp;
import org.firstinspires.ftc.rmrobotics.util.config.Robot;
import org.firstinspires.ftc.rmrobotics.util.config.TheRMite;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

@TeleOp(name="TMecanum", group="TheRMite")
public class TMecanum extends RTeleOp {

    private TheRMite config;

    @Override
    protected void calculate() {
        double forward = control.joystickValue(Controller.ONE, Joystick.LEFT, Axis.Y);
        double strafe = control.joystickValue(Controller.ONE, Joystick.LEFT, Axis.X);
        double rotate = control.joystickValue(Controller.ONE, Joystick.RIGHT, Axis.X);
        double max = maxCheck(forward, strafe, rotate);

        config.FL().setPower((forward + strafe - rotate)/max);
        config.FR().setPower((forward - strafe + rotate)/max);
        config.BL().setPower((forward - strafe - rotate)/max);
        config.BR().setPower((forward + strafe + rotate)/max);

        addTelemetry();
    }

    @Override
    protected Robot setRobot() {
        return config = new TheRMite(hardwareMap);
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

    @Override
    protected void addTelemetry() {
        telemetry.addData("TIME", runtime.time());
        telemetry.addData("FL", config.FL().getPower());
        telemetry.addData("FR", config.FR().getPower());
        telemetry.addData("BL", config.BL().getPower());
        telemetry.addData("BR", config.BR().getPower());
    }
}
