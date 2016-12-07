package org.firstinspires.ftc.rmrobotics.opmodes.feRMilab;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.rmrobotics.control.Axis;
import org.firstinspires.ftc.rmrobotics.control.Button;
import org.firstinspires.ftc.rmrobotics.control.Controller;
import org.firstinspires.ftc.rmrobotics.control.Dpad;
import org.firstinspires.ftc.rmrobotics.control.Joystick;
import org.firstinspires.ftc.rmrobotics.core.RTeleOp;
import org.firstinspires.ftc.rmrobotics.util.DriveUtil;
import org.firstinspires.ftc.rmrobotics.util.Robot;
import org.firstinspires.ftc.rmrobotics.util.config.FeRMilab;
import org.firstinspires.ftc.rmrobotics.util.config.TheRMite;

/**
 * Created by Simon on 11/30/16.
 */

@TeleOp(name = "FTele")
@Disabled
public class FTele extends RTeleOp {

    private ElapsedTime runtime = new ElapsedTime();

    private FeRMilab config;
    private boolean resTog = false;
    private boolean belt = false;

    @Override
    protected void calculate() {
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

        boolean harvest = control.button(Controller.ONE, Button.RBUMP);
        boolean eject = control.button(Controller.ONE, Button.LBUMP);
        if (harvest && eject) {
            config.harvester().setPosition(0.5);
        } else if (harvest) {
            config.harvester().setPosition(0);
            config.harvester().getParent().setDirection(Servo.Direction.FORWARD);
        } else if (eject) {
            config.harvester().setPosition(0);
            config.harvester().getParent().setDirection(Servo.Direction.REVERSE);
        } else {
            config.harvester().setPosition(0.5);
        }

        boolean beltUp = control.dpadValue(Controller.TWO, Dpad.UP);
        boolean beltDown = control.dpadValue(Controller.TWO, Dpad.DOWN);
        if (control.buttonPressed(Controller.TWO, Button.START)) {
            belt = !belt;
        }
        if (belt) {
            config.belt().setPosition(0);
        } else {
            config.belt().setPosition(0.5);
        }
        if (beltUp && beltDown) {
            config.belt().setPosition(0.5);
        } else if (beltUp) {
            config.belt().setPosition(0);
            config.belt().getParent().setDirection(Servo.Direction.FORWARD);
        } else if (beltDown) {
            config.belt().setPosition(0);
            config.belt().getParent().setDirection(Servo.Direction.REVERSE);
        } else {
            config.belt().setPosition(0.5);
        }

        if (control.buttonPressed(Controller.TWO, Button.LBUMP)) {
            config.beaconL().setPosition(0);
        } else {
            config.beaconL().setPosition(1);
        }
        if (control.buttonPressed(Controller.TWO, Button.RBUMP)) {
            config.beaconR().setPosition(1);
        } else {
            config.beaconR().setPosition(0);
        }

        if (control.buttonPressed(Controller.TWO, Button.A)) {
            config.flyL().setPower(1.0);
            config.flyR().setPower(1.0);
        } else if (control.buttonPressed(Controller.TWO, Button.B)) {
            config.flyL().setPower(-1);
            config.flyR().setPower(-1);
        } else {
            config.flyL().setPower(0);
            config.flyR().setPower(0);
        }

        addTelemetry();
    }

    @Override
    protected Robot setRobot() {
        return config = new FeRMilab(hardwareMap, true);
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
