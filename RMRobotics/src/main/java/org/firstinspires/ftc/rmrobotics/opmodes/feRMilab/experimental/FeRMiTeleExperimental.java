package org.firstinspires.ftc.rmrobotics.opmodes.feRMilab.experimental;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.rmrobotics.core.RTele;
import org.firstinspires.ftc.rmrobotics.util.DriveUtil;
import org.firstinspires.ftc.rmrobotics.util.config.FeRMilab;
import org.firstinspires.ftc.rmrobotics.util.config.Robot;

/**
 * Created by Simon on 4/14/2017.
 */

@TeleOp(name = "FeRMiTeleExperimental")
public class FeRMiTeleExperimental extends RTele {
    private FeRMilab config;
    private boolean triggered = false;

    @Override
    protected void calculate() {
        double driveWeight = 1.0;
        double turnWeight = 1.0;
        double res = 1.0;
        boolean resTog = false;
        double[] voltages = DriveUtil.mecanumDrive(gamepad1.left_stick_x, gamepad1.left_stick_y, driveWeight, gamepad1.right_stick_x, turnWeight, res, resTog);
        config.FL().setPower(voltages[0]);
        config.FR().setPower(voltages[1]);
        config.BL().setPower(voltages[2]);
        config.BR().setPower(voltages[3]);

        boolean harvest = gamepad1.right_bumper;
        boolean eject = gamepad1.left_bumper;
        if (harvest && eject) {
            config.belt().setPower(0);
        } else if (harvest) {
            config.belt().setPower(1.0);
        } else if (eject) {
            config.belt().setPower(-1.0);
        } else {
            config.belt().setPower(0);
        }

        if (gamepad2.y) {
            config.flyL().setPower(0.5);
            config.flyR().setPower(0.5);
        } else if (gamepad2.a) {
            config.flyL().setPower(-0.7);
            config.flyR().setPower(-0.7);
        } else if (gamepad2.right_bumper) {
            config.flyL().setPower(1.0);
            config.flyR().setPower(1.0);
        } else {
            config.flyL().setPower(0);
            config.flyR().setPower(0);
        }

        // TODO: find way of determining location of pad relative to starting position
        if(gamepad2.x){
            config.beaconL().setPower(1.0);
        } else if (gamepad2.b){
            config.beaconR().setPower(1.0);
        } else {
            config.beaconL().setPower(0);
            config.beaconR().setPower(0);
        }

        if (gamepad2.left_bumper){
            config.index().setPosition(.5);
        } else {
            config.index().setPosition(.12);
        }

        if (gamepad2.left_trigger > 0.3) {
            config.lift().setPower(-gamepad2.left_trigger);
        } else if (gamepad2.right_trigger > 0.3) {
            config.lift().setPower(gamepad2.right_trigger/2);
        } else {
            config.lift().setPower(0);
        }

        if (gamepad2.dpad_down) {
            config.liftHold().setPosition(1);
            triggered = true;
        } else if (triggered) {
            config.liftHold().setPosition(0);
        } else {
            config.liftHold().setPosition(0.4);
        }

        addTelemetry();
    }

    @Override
    protected Robot setRobot() {
        return config = new FeRMilab(hardwareMap, DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
}
