package org.firstinspires.ftc.rmrobotics.opmodes.histoy.theRMite;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.rmrobotics.control.Axis;
import org.firstinspires.ftc.rmrobotics.control.Button;
import org.firstinspires.ftc.rmrobotics.control.Controller;
import org.firstinspires.ftc.rmrobotics.control.Joystick;
import org.firstinspires.ftc.rmrobotics.core.RTeleOp;
import org.firstinspires.ftc.rmrobotics.util.DriveUtil;
import org.firstinspires.ftc.rmrobotics.util.Robot;
import org.firstinspires.ftc.rmrobotics.util.config.TheRMite;

@TeleOp(name="tMecanum", group="TheRMite")
@Disabled
public class TMecanum extends RTeleOp {

    private TheRMite config;
    private boolean resTog = false;

    @Override
    public void calculate() {
        double forward = control.joystickValue(Controller.ONE, Joystick.LEFT, Axis.Y);
        double strafe = control.joystickValue(Controller.ONE, Joystick.LEFT, Axis.X);
        double rotate = control.joystickValue(Controller.ONE, Joystick.RIGHT, Axis.X);

        double driveWeight = 1.0;
        double turnWeight = 1.0;
        double res = 0.4;
        if (control.buttonPressed(Controller.ONE, Button.BACK)) {
            resTog = !resTog;
        }
        double[] voltages = DriveUtil.mecanumDrive(strafe, forward, driveWeight, rotate, turnWeight, res, resTog);
        config.FL().setPower(voltages[0]);
        config.FR().setPower(voltages[1]);
        config.BL().setPower(voltages[2]);
        config.BR().setPower(voltages[3]);

        addTelemetry();
    }

    @Override
    protected Robot setRobot() {
        return config = new TheRMite(hardwareMap);
    }

    @Override
    protected void addTelemetry() {
        telemetry.addData("TIME", runtime.time());
        telemetry.addData("FL", config.FL().getPower() + " " + config.FL().getCurPos());
        telemetry.addData("FR", config.FR().getPower() + " " + config.FR().getCurPos());
        telemetry.addData("BL", config.BL().getPower() + " " + config.BL().getCurPos());
        telemetry.addData("BR", config.BR().getPower() + " " + config.BR().getCurPos());
    }
}
