package org.firstinspires.ftc.rmrobotics.opmodes.feRMilab.autored;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.rmrobotics.core.FeRMiLinear;
import org.firstinspires.ftc.rmrobotics.util.enums.Color;
import org.firstinspires.ftc.rmrobotics.util.enums.Direction;

import static org.firstinspires.ftc.rmrobotics.util.enums.Drive.TIME;

/**
 * Created by Simon on 2/12/17.
 */
// RED TEAM

@Autonomous(name = "RED: Shoot Ramp")
public class ShootRamp extends FeRMiLinear {
    @Override
    public void runOpMode() throws InterruptedException {
        super.initialize(Color.RED, DcMotor.RunMode.RUN_USING_ENCODER, Direction.FORWARD);

        // voltage = flyMC.getVoltage()*-0.1242 + 2.421 + 0.1;
        telemetry.addData("voltage", voltage);
        telemetry.update();

        // sleep for 10 seconds
        sleep(15000);

        // drive forwards
        drive(TIME, 200, 0.1);
        drive(TIME, 200, 0.3);

        // begin flywheel spin up
        flyL.setPower(1.0);
        flyR.setPower(1.0);
        driveStop(TIME, 1000, 0.5);
        sleep(2000);

        // open indexer
        index.setPosition(0.5);
        sleep(200);

        // turn on belt
        belt.setPower(0.85);
        sleep(2000);
        flyL.setPower(0);
        flyR.setPower(0);
        belt.setPower(0);

        // END SHOOTING

        liftHold.setPosition(0.03);

        // turn at an angle
        turn(Direction.RIGHT, -45, 0.4);

        // drive forward
        driveStop(TIME, 1000, 0.4);

        // face ramp
        turn(Direction.CENTER, -85, 0.4);

        // drive onto ramp
        driveStop(TIME, 1750, 0.4);

        while (opModeIsActive()) {
            addTelemetry();
        }
    }
}